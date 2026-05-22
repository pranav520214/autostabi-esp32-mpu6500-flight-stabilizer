// =============================================================================
//  Firmware.ino  —  AUTOSTABI  IBus Edition
//  ESP32 Fixed-Wing Autostabilization with WebSocket Ground Station
//
//  Hardware:
//    MPU6500  : I2C  SDA=21  SCL=22  (auto-detect 0x68/0x69)
//    IBus RX  : GPIO16  Serial2  115200 baud
//    Servos   : LeftAil=18  RightAil=19  Rudder=23  Elevator=5  FlapAux=4
//    WiFi AP  : SSID="AUTOSTABI"  pass="flysafe123"  IP=192.168.4.1
//    HTTP     : port 80   (serves GroundStation.html)
//    WebSocket: port 81   (JSON telemetry every 50 ms)
//
//  Libraries (Arduino IDE Library Manager):
//    ESP32Servo        >= 0.13.0   (madhephaestus)
//    WebSockets        >= 2.4.1    (Markus Sattler)
//  Built-in (ESP32 core):
//    WiFi, WebServer, Wire, HardwareSerial, esp_task_wdt
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>
#include <esp_idf_version.h>
#include <math.h>

// ─── WiFi / Network ─────────────────────────────────────────────────────────
#define WIFI_SSID      "AUTOSTABI"
#define WIFI_PASS      "flysafe123"
#define HTTP_PORT      80
#define WS_PORT        81

// ─── Servo GPIO pins ─────────────────────────────────────────────────────────
#define PIN_LEFT_AIL   18
#define PIN_RIGHT_AIL  19
#define PIN_RUDDER     23
#define PIN_ELEVATOR    5
#define PIN_FLAP        4

// ─── I2C pins ────────────────────────────────────────────────────────────────
#define I2C_SDA        21
#define I2C_SCL        22

// ─── IBus ────────────────────────────────────────────────────────────────────
#define IBUS_SERIAL    Serial2
#define IBUS_BAUD      115200
#define IBUS_RX_PIN    16
#define IBUS_FRAME     32
#define IBUS_HEADER0   0x20
#define IBUS_HEADER1   0x40
#define IBUS_CH        10
#define IBUS_FS_MS     500    // failsafe if no frame for 500 ms

// ─── Watchdog ────────────────────────────────────────────────────────────────
#define WDT_SEC        3

// ─── Channel indices ─────────────────────────────────────────────────────────
#define CH_AIL   0
#define CH_ELE   1
#define CH_THR   2
#define CH_RUD   3
#define CH_SWA   4
#define CH_SWC   5
#define CH_VRA   6
#define CH_SWB   7
#define CH_SWD   8
#define CH_VRB   9

// ─── Flight modes ────────────────────────────────────────────────────────────
enum FlightMode { MODE_NORMAL = 0, MODE_DIRECT = 1 };

// =============================================================================
//  EMBEDDED GROUND STATION HTML  (served on HTTP port 80)
//  Keep as PROGMEM to save DRAM.
// =============================================================================
static const char GS_HTML[] PROGMEM = R"GSHTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1.0"/>
<title>AUTOSTABI — Ground Station</title>
<style>
:root{
  --bg:#090b0e;--panel:#111520;--panel2:#171d28;--border:#1c2535;
  --amber:#ffb000;--amber-d:#6b4800;--green:#00e676;--green-d:#003d1e;
  --cyan:#00cfff;--red:#ff3b3b;--red-d:#3d0f0f;--yellow:#ffd600;
  --text:#c4d4e0;--dim:#3a4a5a;--mono:'Courier New',monospace;
  --r:5px;--sh:0 4px 20px rgba(0,0,0,.55);
}
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:'Segoe UI',sans-serif;
  min-height:100vh;display:flex;flex-direction:column;overflow-x:hidden}
/* ── Header ── */
header{display:flex;align-items:center;justify-content:space-between;
  padding:8px 18px;background:var(--panel);border-bottom:1px solid var(--border);
  box-shadow:var(--sh);z-index:10;flex-wrap:wrap;gap:8px}
.brand{font-size:1.3rem;font-weight:700;letter-spacing:3px;color:var(--amber);
  font-family:var(--mono)}
.hbtns{display:flex;gap:8px;flex-wrap:wrap}
button{padding:6px 14px;border:1px solid var(--border);border-radius:var(--r);
  background:var(--panel2);color:var(--text);cursor:pointer;font-size:.8rem;
  font-family:var(--mono);letter-spacing:1px;transition:.2s}
button:hover{border-color:var(--amber);color:var(--amber)}
.status-dot{width:10px;height:10px;border-radius:50%;background:#333;
  display:inline-block;margin-right:6px;transition:.3s}
.status-dot.connected{background:var(--green);box-shadow:0 0 8px var(--green)}
.status-dot.connecting{background:var(--yellow);animation:pulse 1s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
.status-txt{font-family:var(--mono);font-size:.8rem}
/* ── Mode badges ── */
.badge{padding:4px 12px;border-radius:3px;font-size:.75rem;font-weight:700;
  font-family:var(--mono);letter-spacing:2px;border:1px solid}
.badge.normal{color:var(--green);border-color:var(--green);background:var(--green-d)}
.badge.direct{color:var(--yellow);border-color:var(--yellow);background:#2a2000}
.badge.crash{color:var(--red);border-color:var(--red);background:var(--red-d);
  animation:pulse .5s infinite}
.badge.failsafe{color:var(--amber);border-color:var(--amber);background:var(--amber-d);
  animation:pulse .7s infinite}
/* ── Main layout ── */
main{flex:1;display:grid;
  grid-template-columns:1fr 320px;
  grid-template-rows:auto auto;
  gap:12px;padding:12px;max-width:1400px;margin:0 auto;width:100%}
@media(max-width:900px){main{grid-template-columns:1fr;}}
/* ── Panel card ── */
.card{background:var(--panel);border:1px solid var(--border);border-radius:var(--r);
  padding:14px;box-shadow:var(--sh)}
.card-title{font-size:.7rem;letter-spacing:3px;color:var(--dim);margin-bottom:10px;
  font-family:var(--mono);text-transform:uppercase}
/* ── Artificial Horizon ── */
.horizon-wrap{display:flex;flex-direction:column;align-items:center;gap:12px}
canvas#ahi{border:2px solid var(--border);border-radius:50%;cursor:pointer;
  box-shadow:0 0 30px rgba(0,207,255,.08)}
.att-readout{display:flex;gap:24px;justify-content:center}
.att-val{text-align:center}
.att-lbl{font-size:.65rem;letter-spacing:2px;color:var(--dim);font-family:var(--mono)}
.att-num{font-size:1.6rem;font-family:var(--mono);color:var(--cyan)}
/* ── Right column ── */
.right-col{display:flex;flex-direction:column;gap:12px}
/* ── Servo bars ── */
.bars-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.bar-row{display:flex;flex-direction:column;gap:4px}
.bar-lbl{font-size:.65rem;font-family:var(--mono);color:var(--dim);
  letter-spacing:1px;display:flex;justify-content:space-between}
.bar-track{height:8px;background:#0d1117;border-radius:4px;overflow:hidden;
  border:1px solid var(--border)}
.bar-fill{height:100%;border-radius:4px;transition:width .1s;
  background:linear-gradient(90deg,var(--cyan),var(--amber))}
/* ── Channel bars ── */
.ch-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}
.ch-item{display:flex;flex-direction:column;gap:3px}
/* ── VRA slider ── */
.vra-row{display:flex;align-items:center;gap:10px}
.vra-row input[type=range]{flex:1;accent-color:var(--amber)}
.vra-val{font-family:var(--mono);font-size:.9rem;color:var(--amber);min-width:36px}
/* ── Status row ── */
.status-row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.sw-badge{padding:3px 8px;border-radius:3px;font-size:.65rem;font-family:var(--mono);
  letter-spacing:1px;border:1px solid var(--dim);color:var(--dim)}
.sw-badge.on{border-color:var(--amber);color:var(--amber);background:var(--amber-d)}
.ts-txt{font-family:var(--mono);font-size:.65rem;color:var(--dim);margin-left:auto}
/* ── Graph ── */
canvas#graph{width:100%;height:120px;display:block;border-radius:var(--r)}
/* ── Bottom row ── */
.bottom-row{grid-column:1/-1}
</style>
</head>
<body>
<header>
  <span class="brand">✈ AUTOSTABI</span>
  <div class="hbtns">
    <span><span class="status-dot" id="sdot"></span><span class="status-txt" id="stxt">DISCONNECTED</span></span>
    <button onclick="wsConnect()">BIND / RECONNECT</button>
    <button onclick="resetHorizon()">RESET HORIZON</button>
    <button onclick="toggleMode()">TOGGLE MODE</button>
  </div>
  <div style="display:flex;gap:8px;align-items:center">
    <span class="badge normal" id="modeBadge">NORMAL</span>
    <span class="badge crash" id="crashBadge" style="display:none">CRASH PREV</span>
    <span class="badge failsafe" id="fsBadge" style="display:none">FAILSAFE</span>
  </div>
</header>

<main>
  <!-- Left: Artificial Horizon + Graph -->
  <div style="display:flex;flex-direction:column;gap:12px">
    <div class="card">
      <div class="card-title">Attitude Indicator</div>
      <div class="horizon-wrap">
        <canvas id="ahi" width="360" height="360"></canvas>
        <div class="att-readout">
          <div class="att-val">
            <div class="att-lbl">ROLL</div>
            <div class="att-num" id="rollNum">0.0°</div>
          </div>
          <div class="att-val">
            <div class="att-lbl">PITCH</div>
            <div class="att-num" id="pitchNum">0.0°</div>
          </div>
          <div class="att-val">
            <div class="att-lbl">YAW RATE</div>
            <div class="att-num" id="yawNum">0.0°/s</div>
          </div>
        </div>
      </div>
    </div>
    <div class="card bottom-row">
      <div class="card-title">Roll / Pitch — 10s History</div>
      <canvas id="graph" width="800" height="120"></canvas>
    </div>
  </div>

  <!-- Right column -->
  <div class="right-col">
    <div class="card">
      <div class="card-title">Servo Positions</div>
      <div class="bars-grid">
        <div class="bar-row">
          <div class="bar-lbl"><span>LEFT AIL</span><span id="lsVal">—</span></div>
          <div class="bar-track"><div class="bar-fill" id="lsBar" style="width:50%"></div></div>
        </div>
        <div class="bar-row">
          <div class="bar-lbl"><span>RIGHT AIL</span><span id="rsVal">—</span></div>
          <div class="bar-track"><div class="bar-fill" id="rsBar" style="width:50%"></div></div>
        </div>
        <div class="bar-row">
          <div class="bar-lbl"><span>ELEVATOR</span><span id="esVal">—</span></div>
          <div class="bar-track"><div class="bar-fill" id="esBar" style="width:50%"></div></div>
        </div>
        <div class="bar-row">
          <div class="bar-lbl"><span>RUDDER</span><span id="rvVal">—</span></div>
          <div class="bar-track"><div class="bar-fill" id="rvBar" style="width:50%"></div></div>
        </div>
      </div>
    </div>

    <div class="card">
      <div class="card-title">RC Channels (1000–2000)</div>
      <div class="ch-grid" id="chGrid"></div>
    </div>

    <div class="card">
      <div class="card-title">VRA Sensitivity</div>
      <div class="vra-row">
        <input type="range" id="vraSlider" min="0" max="100" value="50"
          oninput="sendVRA(this.value)">
        <span class="vra-val" id="vraVal">50%</span>
      </div>
    </div>

    <div class="card">
      <div class="card-title">Switch States &amp; Last Update</div>
      <div class="status-row">
        <span class="sw-badge" id="swA">SWA</span>
        <span class="sw-badge" id="swB">SWB</span>
        <span class="sw-badge" id="swC">SWC</span>
        <span class="sw-badge" id="swD">SWD</span>
        <span class="ts-txt" id="tsText">—</span>
      </div>
    </div>
  </div>
</main>

<script>
// ── WebSocket ──────────────────────────────────────────────────────────────
let ws = null, retryDelay = 1000, retryTimer = null;
const WS_URL = 'ws://192.168.4.1:81';

function wsConnect() {
  clearTimeout(retryTimer);
  if (ws) { ws.close(); ws = null; }
  setStatus('connecting');
  ws = new WebSocket(WS_URL);
  ws.onopen  = () => { setStatus('connected'); retryDelay = 1000; };
  ws.onclose = () => { setStatus('disconnected'); scheduleRetry(); };
  ws.onerror = () => { ws.close(); };
  ws.onmessage = e => { try { handleTelemetry(JSON.parse(e.data)); } catch(_) {} };
}
function scheduleRetry() {
  retryTimer = setTimeout(() => { wsConnect(); retryDelay = Math.min(retryDelay * 1.6, 16000); }, retryDelay);
}
function wsSend(obj) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj)); }
function setStatus(s) {
  const dot = document.getElementById('sdot'), txt = document.getElementById('stxt');
  dot.className = 'status-dot ' + s;
  txt.textContent = s.toUpperCase();
}
function toggleMode() { wsSend({cmd:'toggle_mode'}); }
function sendVRA(v)   { wsSend({cmd:'set_vra', val: String(v)}); }
function resetHorizon() { rollOffset = state.roll || 0; pitchOffset = state.pitch || 0; }

// ── Telemetry state ────────────────────────────────────────────────────────
let state = {r:0,p:0,y:0,ls:1500,rs:1500,es:1500,rsv:1500,
             c1:1500,c2:1500,c3:1000,c4:1500,c5:1000,c6:1000,c7:1500,c8:1000,c9:1000,
             vra:50,mode:'NORMAL',crash:false,fs:false,swA:false,swB:false,swC:false,swD:false};
let rollOffset = 0, pitchOffset = 0;

const CH_LABELS = ['CH1 AIL','CH2 ELE','CH3 THR','CH4 RUD','CH5 SWA','CH6 SWC','CH7 VRA','CH8 SWB','CH9 SWD'];
const CH_KEYS   = ['c1','c2','c3','c4','c5','c6','c7','c8','c9'];

// Build channel grid
const chGrid = document.getElementById('chGrid');
CH_LABELS.forEach((lbl, i) => {
  chGrid.innerHTML += `<div class="ch-item">
    <div class="bar-lbl"><span>${lbl}</span><span id="cv${i}">—</span></div>
    <div class="bar-track"><div class="bar-fill" id="cb${i}" style="width:50%"></div></div>
  </div>`;
});

function handleTelemetry(d) {
  Object.assign(state, d);
  const r = (d.r !== undefined ? d.r : state.r) - rollOffset;
  const p = (d.p !== undefined ? d.p : state.p) - pitchOffset;

  document.getElementById('rollNum').textContent  = r.toFixed(1) + '°';
  document.getElementById('pitchNum').textContent = p.toFixed(1) + '°';
  document.getElementById('yawNum').textContent   = (state.y||0).toFixed(1) + '°/s';

  // Servo bars
  updateBar('ls', state.ls); updateBar('rs', state.rs);
  updateBar('es', state.es); updateBar('rv', state.rsv);

  // Channel bars
  CH_KEYS.forEach((k, i) => {
    const v = state[k] || 1500;
    const pct = ((v - 1000) / 1000 * 100).toFixed(0);
    const el = document.getElementById('cb'+i);
    if (el) el.style.width = pct + '%';
    const vl = document.getElementById('cv'+i);
    if (vl) vl.textContent = v;
  });

  // VRA slider
  if (d.vra !== undefined) {
    document.getElementById('vraSlider').value = d.vra;
    document.getElementById('vraVal').textContent = d.vra + '%';
  }

  // Mode badge
  const mb = document.getElementById('modeBadge');
  const mode = (state.mode||'NORMAL').toUpperCase();
  mb.textContent = mode;
  mb.className = 'badge ' + (mode === 'DIRECT' ? 'direct' : 'normal');

  // Crash badge
  document.getElementById('crashBadge').style.display = state.crash ? '' : 'none';
  document.getElementById('fsBadge').style.display    = state.fs    ? '' : 'none';

  // Switches
  ['A','B','C','D'].forEach(sw => {
    const el = document.getElementById('sw'+sw);
    if (el) el.className = 'sw-badge' + (state['sw'+sw] ? ' on' : '');
  });

  document.getElementById('tsText').textContent = 'Last: ' + new Date().toLocaleTimeString();

  // Graph
  const now = Date.now();
  graphPush(now, r, p);
}

function updateBar(id, us) {
  const pct = ((us - 1000) / 1000 * 100).toFixed(0);
  const el = document.getElementById(id+'Bar');
  if (el) el.style.width = pct + '%';
  const vl = document.getElementById(id+'Val');
  if (vl) vl.textContent = us + 'µs';
}

// ── Artificial Horizon ────────────────────────────────────────────────────
const ahi = document.getElementById('ahi');
const ctx = ahi.getContext('2d');
const AW = ahi.width, AH = ahi.height, CX = AW/2, CY = AH/2, R = AW/2 - 2;

function drawAHI() {
  const roll  = ((state.r||0) - rollOffset) * Math.PI / 180;
  const pitch = ((state.p||0) - pitchOffset);
  const pitchPx = pitch * (AH / 90);

  ctx.clearRect(0, 0, AW, AH);
  ctx.save();
  ctx.beginPath(); ctx.arc(CX, CY, R, 0, Math.PI*2); ctx.clip();

  // Sky
  ctx.save();
  ctx.translate(CX, CY);
  ctx.rotate(roll);
  ctx.translate(0, pitchPx);
  ctx.fillStyle = '#1a4a8a';
  ctx.fillRect(-AW, -AH, AW*2, AH);
  // Ground
  ctx.fillStyle = '#5c3a1a';
  ctx.fillRect(-AW, 0, AW*2, AH);
  // Horizon line
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(-AW, 0); ctx.lineTo(AW, 0); ctx.stroke();
  // Pitch lines
  ctx.strokeStyle = 'rgba(255,255,255,0.5)';
  ctx.lineWidth = 1;
  ctx.font = '11px Courier New';
  ctx.fillStyle = '#fff';
  ctx.textAlign = 'center';
  for (let deg = -60; deg <= 60; deg += 10) {
    if (deg === 0) continue;
    const y = -deg * (AH / 90);
    const w = deg % 20 === 0 ? 50 : 30;
    ctx.beginPath(); ctx.moveTo(-w, y); ctx.lineTo(w, y); ctx.stroke();
    if (deg % 20 === 0) ctx.fillText(deg, w + 14, y + 4);
  }
  ctx.restore();

  // Aircraft reticle (fixed)
  ctx.strokeStyle = '#ffb000';
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(CX-60, CY); ctx.lineTo(CX-20, CY); ctx.lineTo(CX-20, CY+12);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(CX+60, CY); ctx.lineTo(CX+20, CY); ctx.lineTo(CX+20, CY+12);
  ctx.stroke();
  ctx.beginPath(); ctx.arc(CX, CY, 4, 0, Math.PI*2); ctx.stroke();

  // Roll arc + tick marks
  ctx.save();
  ctx.translate(CX, CY);
  ctx.strokeStyle = 'rgba(255,255,255,0.4)';
  ctx.lineWidth = 1;
  for (const angle of [-60,-45,-30,-20,-10,0,10,20,30,45,60]) {
    const a = (angle - 90) * Math.PI/180;
    const len = angle % 30 === 0 ? 14 : 8;
    ctx.beginPath();
    ctx.moveTo(Math.cos(a)*( R-2),   Math.sin(a)*(R-2));
    ctx.lineTo(Math.cos(a)*(R-2-len),Math.sin(a)*(R-2-len));
    ctx.stroke();
  }
  // Roll pointer
  ctx.rotate(roll);
  ctx.fillStyle = '#ffb000';
  ctx.beginPath(); ctx.moveTo(0, -(R-2)); ctx.lineTo(-6, -(R-18)); ctx.lineTo(6, -(R-18)); ctx.closePath(); ctx.fill();
  ctx.restore();

  // Bezel ring
  ctx.beginPath(); ctx.arc(CX, CY, R, 0, Math.PI*2);
  ctx.strokeStyle = '#1c2535'; ctx.lineWidth = 3; ctx.stroke();

  ctx.restore();
  requestAnimationFrame(drawAHI);
}
drawAHI();

// ── Roll/Pitch graph ──────────────────────────────────────────────────────
const gCanvas = document.getElementById('graph');
const gCtx = gCanvas.getContext('2d');
const HISTORY_MS = 10000;
let rollHist = [], pitchHist = [];

function graphPush(ts, r, p) {
  const cutoff = ts - HISTORY_MS;
  rollHist.push([ts, r]);
  pitchHist.push([ts, p]);
  rollHist  = rollHist.filter(pt => pt[0] >= cutoff);
  pitchHist = pitchHist.filter(pt => pt[0] >= cutoff);
  drawGraph();
}

function drawGraph() {
  const W = gCanvas.offsetWidth || gCanvas.width;
  const H = gCanvas.height;
  gCanvas.width = W;
  gCtx.clearRect(0, 0, W, H);

  // Background
  gCtx.fillStyle = '#0d1117';
  gCtx.fillRect(0, 0, W, H);

  // Grid lines
  gCtx.strokeStyle = '#1c2535'; gCtx.lineWidth = 1;
  for (let deg = -60; deg <= 60; deg += 30) {
    const y = H/2 - (deg / 90) * (H/2);
    gCtx.beginPath(); gCtx.moveTo(0, y); gCtx.lineTo(W, y); gCtx.stroke();
  }
  gCtx.strokeStyle = '#2a3545';
  for (let i = 0; i <= 10; i++) {
    const x = (i / 10) * W;
    gCtx.beginPath(); gCtx.moveTo(x, 0); gCtx.lineTo(x, H); gCtx.stroke();
  }

  // Labels
  gCtx.fillStyle = '#3a4a5a'; gCtx.font = '10px Courier New';
  gCtx.fillText('90°', 2, H/2 - (90/90)*(H/2) + 12);
  gCtx.fillText('0°',  2, H/2 + 4);
  gCtx.fillText('-90°',2, H/2 + (90/90)*(H/2));

  const now = Date.now();

  function drawLine(hist, color) {
    if (hist.length < 2) return;
    gCtx.strokeStyle = color; gCtx.lineWidth = 1.5;
    gCtx.beginPath();
    hist.forEach((pt, i) => {
      const x = ((pt[0] - (now - HISTORY_MS)) / HISTORY_MS) * W;
      const y = H/2 - (pt[1] / 90) * (H/2);
      i === 0 ? gCtx.moveTo(x, y) : gCtx.lineTo(x, y);
    });
    gCtx.stroke();
  }
  drawLine(rollHist,  '#00cfff');
  drawLine(pitchHist, '#ffb000');

  // Legend
  gCtx.fillStyle = '#00cfff'; gCtx.fillRect(W-80, 6, 12, 3);
  gCtx.fillStyle = '#3a4a5a'; gCtx.font = '10px Courier New'; gCtx.fillText('ROLL',  W-64, 10);
  gCtx.fillStyle = '#ffb000'; gCtx.fillRect(W-80, 14, 12, 3);
  gCtx.fillStyle = '#3a4a5a'; gCtx.fillText('PITCH', W-64, 18);
}

// Auto-connect on load
wsConnect();
</script>
</body>
</html>
)GSHTML";

// =============================================================================
//  IBus Receiver
// =============================================================================
class IBusReceiver {
public:
  uint16_t ch[IBUS_CH];   // channel values 1000-2000
  bool     failsafe = false;

  IBusReceiver() {
    for (int i = 0; i < IBUS_CH; i++) ch[i] = (i == CH_THR) ? 1000 : 1500;
  }

  void begin() {
    IBUS_SERIAL.begin(IBUS_BAUD, SERIAL_8N1, IBUS_RX_PIN, -1);
    _lastFrameMs = millis();
    Serial.println(F("[IBus] Serial2 started on GPIO16"));
  }

  // Call every loop — reads bytes, assembles frames, parses channels
  void update() {
    while (IBUS_SERIAL.available()) {
      uint8_t b = IBUS_SERIAL.read();
      if (_bufLen == 0) {
        if (b != IBUS_HEADER0) continue;  // wait for 0x20
      } else if (_bufLen == 1) {
        if (b != IBUS_HEADER1) { _bufLen = 0; continue; } // expect 0x40
      }
      _buf[_bufLen++] = b;
      if (_bufLen == IBUS_FRAME) {
        if (verifyChecksum()) {
          parseChannels();
          _lastFrameMs = millis();
        }
        _bufLen = 0;
      }
    }
    // Signal loss detection
    failsafe = (millis() - _lastFrameMs) > IBUS_FS_MS;
    if (failsafe) applyFailsafe();
  }

private:
  uint8_t  _buf[IBUS_FRAME];
  uint8_t  _bufLen   = 0;
  uint32_t _lastFrameMs = 0;

  // IBus checksum: bytes[30:31] = 0xFFFF - sum(bytes[0..29])
  bool verifyChecksum() {
    uint16_t sum = 0;
    for (int i = 0; i < 30; i++) sum += _buf[i];
    uint16_t chk = 0xFFFF - sum;
    uint16_t got = (uint16_t)_buf[30] | ((uint16_t)_buf[31] << 8);
    return chk == got;
  }

  void parseChannels() {
    // Bytes 2..29 = 14 channels × 2 bytes little-endian; we use first 10
    for (int i = 0; i < IBUS_CH; i++) {
      int idx = 2 + i * 2;
      uint16_t v = (uint16_t)_buf[idx] | ((uint16_t)_buf[idx+1] << 8);
      if (v >= 800 && v <= 2200) ch[i] = constrain(v, 1000, 2000);
    }
  }

  void applyFailsafe() {
    ch[CH_THR] = 1300;  // 30% throttle
    ch[CH_AIL] = 1500; ch[CH_ELE] = 1500;
    ch[CH_RUD] = 1500;
  }
};

// =============================================================================
//  Kalman Filter (1-axis)
// =============================================================================
class KalmanAxis {
public:
  KalmanAxis(float qa=0.001f, float qb=0.003f, float rm=0.03f)
    : _qa(qa), _qb(qb), _rm(rm) {}

  void reset(float initAngle=0) {
    _angle=initAngle; _bias=0;
    _cov[0][0]=1; _cov[0][1]=0; _cov[1][0]=0; _cov[1][1]=1;
  }

  float update(float gyroRate, float accelAngle, float dt) {
    float rate = gyroRate - _bias;
    _angle += rate * dt;
    _cov[0][0] += dt*(dt*_cov[1][1] - _cov[0][1] - _cov[1][0] + _qa);
    _cov[0][1] -= dt*_cov[1][1];
    _cov[1][0] -= dt*_cov[1][1];
    _cov[1][1] += _qb*dt;
    float S  = _cov[0][0] + _rm;
    float K0 = _cov[0][0]/S, K1 = _cov[1][0]/S;
    float y  = accelAngle - _angle;
    _angle += K0*y; _bias += K1*y;
    float p00 = _cov[0][0], p01 = _cov[0][1];
    _cov[0][0] -= K0*p00; _cov[0][1] -= K0*p01;
    _cov[1][0] -= K1*p00; _cov[1][1] -= K1*p01;
    return _angle;
  }
  float angle() const { return _angle; }

private:
  float _qa, _qb, _rm, _angle=0, _bias=0, _cov[2][2]={{1,0},{0,1}};
};

// =============================================================================
//  MPU6500 Driver
// =============================================================================
#define MPU_ADDR_A      0x68
#define MPU_ADDR_B      0x69
#define MPU_WHO_AM_I    0x75
#define MPU_WHO_VAL     0x70
#define MPU_PWR_MGMT_1  0x6B
#define MPU_CONFIG      0x1A
#define MPU_GYRO_CFG    0x1B
#define MPU_ACCEL_CFG   0x1C
#define MPU_ACCEL_CFG2  0x1D
#define MPU_SMPLRT_DIV  0x19
#define MPU_ACCEL_XOUT  0x3B
#define MPU_GYRO_XOUT   0x43
#define ACCEL_SCALE     (1.0f/4096.0f)  // ±8g  → 4096 LSB/g
#define GYRO_SCALE      (1.0f/32.8f)    // ±1000dps → 32.8 LSB/(°/s)
#define CALIB_N         500

class MPU6500 {
public:
  float roll=0, pitch=0;
  float rollRate=0, pitchRate=0, yawRate=0;
  float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
  bool  healthy=false;

  bool begin() {
    _addr=0;
    for (uint8_t a : {MPU_ADDR_A, MPU_ADDR_B}) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission()==0 && readByte(a,MPU_WHO_AM_I)==MPU_WHO_VAL) {
        _addr=a; break;
      }
    }
    if (!_addr) { Serial.println(F("[IMU] MPU6500 not found")); return false; }
    Serial.printf("[IMU] MPU6500 @ 0x%02X\n", _addr);

    writeByte(_addr, MPU_PWR_MGMT_1, 0x01); delay(100);
    writeByte(_addr, MPU_CONFIG,      0x03);  // DLPF ~42Hz
    writeByte(_addr, MPU_GYRO_CFG,    0x10);  // ±1000°/s
    writeByte(_addr, MPU_ACCEL_CFG,   0x10);  // ±8g
    writeByte(_addr, MPU_ACCEL_CFG2,  0x03);
    writeByte(_addr, MPU_SMPLRT_DIV,  0x00);
    delay(150);

    calibrate();
    healthy=true;

    // Init Kalman
    int16_t raw[7]={};
    if (readRaw(raw)) {
      float initR = atan2f(raw[1]*ACCEL_SCALE, raw[2]*ACCEL_SCALE)*57.2958f;
      float initP = atan2f(-raw[0]*ACCEL_SCALE,
                    sqrtf(raw[1]*ACCEL_SCALE*raw[1]*ACCEL_SCALE +
                          raw[2]*ACCEL_SCALE*raw[2]*ACCEL_SCALE))*57.2958f;
      _kRoll.reset(initR); _kPitch.reset(initP);
    }
    _lastUs=micros();
    return true;
  }

  void update() {
    if (!_addr) return;
    int16_t raw[7]={};
    bool ok=false;
    for (int t=0; t<3 && !ok; t++) { ok=readRaw(raw); if (!ok) delayMicroseconds(200); }
    if (!ok) { _fails++; if (_fails>20) healthy=false; return; }
    _fails=0; healthy=true;

    ax = raw[0]*ACCEL_SCALE;
    ay = raw[1]*ACCEL_SCALE;
    az = raw[2]*ACCEL_SCALE;
    gx = raw[4]*GYRO_SCALE - _bx;
    gy = raw[5]*GYRO_SCALE - _by;
    gz = raw[6]*GYRO_SCALE - _bz;

    float aRoll  = atan2f(ay,az)*57.2958f;
    float aPitch = atan2f(-ax,sqrtf(ay*ay+az*az))*57.2958f;

    uint32_t now = micros();
    float dt = (now-_lastUs)*1e-6f;
    _lastUs=now;
    if (dt<=0||dt>0.05f) dt=0.005f;
    if (dt < 0.001f) dt = 0.005f;

    roll      = _kRoll.update(gx, aRoll,  dt);
    pitch     = _kPitch.update(gy,aPitch, dt);
    rollRate  = gx; pitchRate=gy; yawRate=gz;
  }

private:
  uint8_t   _addr=0;
  uint8_t   _fails=0;
  uint32_t  _lastUs=0;
  float     _bx=0,_by=0,_bz=0;
  KalmanAxis _kRoll, _kPitch;

  void calibrate() {
    Serial.println(F("[IMU] Calibrating — hold still..."));
    double sx=0,sy=0,sz=0; int16_t b[7]={};
    for (int i=0;i<CALIB_N;i++) { if (readRaw(b)){sx+=b[4];sy+=b[5];sz+=b[6];} delay(2); }
    _bx=(float)(sx/CALIB_N)*GYRO_SCALE;
    _by=(float)(sy/CALIB_N)*GYRO_SCALE;
    _bz=(float)(sz/CALIB_N)*GYRO_SCALE;
    Serial.printf("[IMU] Gyro bias: %.3f %.3f %.3f\n",_bx,_by,_bz);
  }

  bool readRaw(int16_t out[7]) {
    Wire.beginTransmission(_addr);
    Wire.write(MPU_ACCEL_XOUT);
    if (Wire.endTransmission(false)!=0) return false;
    if (Wire.requestFrom((uint8_t)_addr,(uint8_t)14)!=14) return false;
    for (int i=0;i<7;i++){
      uint8_t hi=Wire.read(),lo=Wire.read();
      out[i]=(int16_t)((hi<<8)|lo);
    }
    return true;
  }
  uint8_t readByte(uint8_t a,uint8_t r){
    Wire.beginTransmission(a); Wire.write(r); Wire.endTransmission(false);
    Wire.requestFrom(a,(uint8_t)1);
    return Wire.available()?Wire.read():0;
  }
  void writeByte(uint8_t a,uint8_t r,uint8_t d){
    Wire.beginTransmission(a); Wire.write(r); Wire.write(d); Wire.endTransmission();
  }
};

// =============================================================================
//  Flight Controller
// =============================================================================
class FlightController {
public:
  // Telemetry outputs
  float   t_roll=0,t_pitch=0,t_yaw=0;
  int     t_ls=1500,t_rs=1500,t_es=1500,t_rv=1500,t_fl=1500;
  int     t_ch[IBUS_CH]={};
  float   t_vra=50;
  bool    t_crash=false, t_fs=false;
  bool    t_swA=false,t_swB=false,t_swC=false,t_swD=false;
  String  t_mode="NORMAL";
  FlightMode t_flMode=MODE_NORMAL;

  void begin(MPU6500* imu, IBusReceiver* rx) {
    _imu=imu; _rx=rx;
    ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
    _sLA.setPeriodHertz(50); _sRA.setPeriodHertz(50);
    _sRD.setPeriodHertz(50); _sEL.setPeriodHertz(50);
    _sFL.setPeriodHertz(50);
    _sLA.attach(PIN_LEFT_AIL,  1000,2000);
    _sRA.attach(PIN_RIGHT_AIL, 1000,2000);
    _sRD.attach(PIN_RUDDER,    1000,2000);
    _sEL.attach(PIN_ELEVATOR,  1000,2000);
    _sFL.attach(PIN_FLAP,      1000,2000);
    writeServos(1500,1500,1500,1500,1500);
    Serial.println(F("[FC] FlightController ready"));
  }

  void computeControl(float dt, uint32_t now) {
    (void)dt;
    auto& rx = *_rx;
    auto& imu= *_imu;

    float roll  = imu.roll,  pitch  = imu.pitch;
    float rRate = imu.rollRate, pRate=imu.pitchRate, yRate=imu.yawRate;

    // Channel read
    uint16_t* c = rx.ch;
    float ailStk = (c[CH_AIL]-1500)/500.0f;  // -1..1
    float eleStk = (c[CH_ELE]-1500)/500.0f;
    float rudStk = (c[CH_RUD]-1500)/500.0f;
    float thr    = (c[CH_THR]-1000)/1000.0f; // 0..1
    float vra    = (c[CH_VRA]-1000)/1000.0f; // 0..1
    bool  swA    = c[CH_SWA]>1700;
    bool  swB    = c[CH_SWB]>1700;
    bool  swC    = c[CH_SWC]>1700;
    bool  swD    = c[CH_SWD]>1700;

    // SWB 1-second long-press → toggle mode
    handleModeToggle(swB, now);

    // SWA landing: elev trim +80µs, throttle cap 1500
    float elevTrim   = 0;
    float thrCap     = 2000;
    float ailDroop   = 0;
    if (swA) { elevTrim=80; thrCap=1500; ailDroop=200; }
    if (swA && pRate < -15.0f) elevTrim += 100;

    // SWC takeoff: elevator +200 for 1 second then release
    if (swC && !_swCPrev) { _swCStartMs=now; _swCActive=true; }
    if (!swC) _swCActive=false;
    if (_swCActive && (now-_swCStartMs)<1000) { elevTrim+=200; thr=0.8f; }
    _swCPrev=swC;

    // SWD flap: ailerons droop 200µs down
    if (swD) ailDroop=200;

    // Failsafe override
    bool fs = rx.failsafe;
    if (fs) {
      ailStk=0; eleStk=0; rudStk=0; thr=0.3f;
    }

    // VRA gain 0-1.5
    float gain = vra * 1.5f;

    // DIRECT passthrough
    if (_mode == MODE_DIRECT) {
      int la = (int)(1500 + ailStk*500);
      int ra = 1500 - (la - 1500);
      if (swD) { la = constrain((int)(la - ailDroop), 1000, 2000); ra = constrain((int)(ra + ailDroop), 1000, 2000); }
      int el = (int)(1500 + eleStk*500);
      int rd = (int)(1500 + rudStk*500);
      setServoTargets(la,ra,rd,el,1500);
      storeTelemetry(roll,pitch,yRate,la,ra,el,rd,1500,c,vra*100,false,fs,swA,swB,swC,swD,"DIRECT");
      return;
    }

    // NORMAL autostabilization
    // angle_error = target(0) - current
    float rollErr  = -roll;   // target 0
    float pitchErr = -pitch;  // target 0

    float rollCorr  = rollErr  * 1.5f * gain;
    float pitchCorr = pitchErr * 1.5f * gain;

    // Gyro damping: rate > 250°/s → reduce gain 50% for 0.5s
    if (fabsf(rRate)>250.0f||fabsf(pRate)>250.0f) _dampUntil=now+500;
    if (now<_dampUntil) { rollCorr*=0.5f; pitchCorr*=0.5f; }

    // Crash prevention: roll>70° or pitch>45° → 100% correction
    bool crash=false;
    if (fabsf(roll)>70.0f) {
      crash=true;
      rollCorr = constrain(-roll*0.02f, -1.0f, 1.0f)*500;
    }
    if (fabsf(pitch)>45.0f) {
      crash=true;
      pitchCorr = constrain(-pitch*0.02f, -1.0f, 1.0f)*500;
    }

    // Servo: stick + correction, in µs offset from 1500
    float ailUs  = (crash ? 0 : ailStk*500) + rollCorr;
    float eleUs  = (crash ? 0 : eleStk*500) + pitchCorr + elevTrim;
    float rudUs  = rudStk*500;

    // Mirror right aileron
    int la = constrain((int)(1500 + ailUs),           1000,2000);
    int ra = constrain((int)(1500 - ailUs + ailDroop), 1000,2000);
    int el = constrain((int)(1500 + eleUs),            1000,2000);
    int rd = constrain((int)(1500 + rudUs),            1000,2000);

    // Throttle cap
    int thrUs = constrain((int)(1000+thr*1000), 1000, (int)thrCap);

    // SWD: both ailerons -200 (droop 20% down)
    if (swD) {
      la = constrain(la-200,1000,2000);
      ra = constrain(ra-200,1000,2000);
    }

    setServoTargets(la,ra,rd,el,1500);

    String modeStr = "NORMAL";
    if (crash) modeStr="RECOVER";
    else if (fs) modeStr="FAILSAFE";
    else if (swA) modeStr="LANDING";
    else if (_swCActive) modeStr="TAKEOFF";

    storeTelemetry(roll,pitch,yRate,la,ra,el,rd,thrUs,c,vra*100,crash,fs,swA,swB,swC,swD,modeStr);
  }

  void applyServos() {
    writeServos(_cmdLA,_cmdRA,_cmdRD,_cmdEL,_cmdFL);
  }

  // From WebSocket command
  void handleCmd(const String& cmd, const String& val) {
    if (cmd=="toggle_mode") {
      _mode = (_mode==MODE_NORMAL) ? MODE_DIRECT : MODE_NORMAL;
      Serial.printf("[FC] Mode: %s\n", _mode==MODE_DIRECT?"DIRECT":"NORMAL");
    } else if (cmd=="set_level") {
      _mode = (val=="2") ? MODE_DIRECT : MODE_NORMAL;
    } else if (cmd=="reset_pid") {
      // (no PID integral state in this simplified controller)
    }
  }

private:
  MPU6500*     _imu=nullptr;
  IBusReceiver*_rx=nullptr;
  Servo        _sLA,_sRA,_sRD,_sEL,_sFL;
  FlightMode   _mode=MODE_NORMAL;
  uint32_t     _dampUntil=0;
  uint32_t     _swBPressMs=0;
  bool         _swBPrev=false, _swBProcessed=false;
  bool         _swCPrev=false, _swCActive=false;
  uint32_t     _swCStartMs=0;
  int          _cmdLA=1500,_cmdRA=1500,_cmdRD=1500,_cmdEL=1500,_cmdFL=1500;

  void handleModeToggle(bool swBNow, uint32_t now) {
    if (swBNow && !_swBPrev) { _swBPressMs=now; _swBProcessed=false; }
    if (swBNow && !_swBProcessed && (now-_swBPressMs)>=1000) {
      _mode = (_mode==MODE_NORMAL) ? MODE_DIRECT : MODE_NORMAL;
      _swBProcessed=true;
      Serial.printf("[FC] Mode toggled: %s\n", _mode==MODE_DIRECT?"DIRECT":"NORMAL");
    }
    _swBPrev=swBNow;
  }

  void writeServos(int la,int ra,int rd,int el,int fl) {
    _sLA.writeMicroseconds(la); _sRA.writeMicroseconds(ra);
    _sRD.writeMicroseconds(rd); _sEL.writeMicroseconds(el);
    _sFL.writeMicroseconds(fl);
  }

  void setServoTargets(int la,int ra,int rd,int el,int fl) {
    _cmdLA=la; _cmdRA=ra; _cmdRD=rd; _cmdEL=el; _cmdFL=fl;
  }

  void storeTelemetry(float r,float p,float y,int la,int ra,int el,int rd,int fl,
                      uint16_t* c, float vra, bool crash, bool fs,
                      bool swA,bool swB,bool swC,bool swD, String modeStr) {
    t_roll=r; t_pitch=p; t_yaw=y;
    t_ls=la; t_rs=ra; t_es=el; t_rv=rd; t_fl=fl;
    for (int i=0;i<IBUS_CH;i++) t_ch[i]=c[i];
    t_vra=vra; t_crash=crash; t_fs=fs;
    t_swA=swA; t_swB=swB; t_swC=swC; t_swD=swD;
    t_mode=modeStr; t_flMode=_mode;
  }
};

// =============================================================================
//  Global instances
// =============================================================================
IBusReceiver  ibus;
MPU6500       imu;
FlightController fc;
WebServer     httpServer(HTTP_PORT);
WebSocketsServer wsServer(WS_PORT);

// Timing
uint32_t lastImuMs=0, lastServoMs=0, lastWsMs=0;
uint32_t imuUnhealthySinceMs=0;

// =============================================================================
//  WebSocket event handler
// =============================================================================
void wsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  if (type==WStype_TEXT && len>0) {
    String s = String((char*)payload);
    // Simple JSON parse for {"cmd":"...","val":"..."}
    String cmd="", val="";
    int ci=s.indexOf("\"cmd\":\"");
    if (ci>=0) { int st=ci+7; int en=s.indexOf('"',st); cmd=s.substring(st,en); }
    int vi=s.indexOf("\"val\":\"");
    if (vi>=0) { int st=vi+7; int en=s.indexOf('"',st); val=s.substring(st,en); }
    if (cmd.length()) fc.handleCmd(cmd,val);
  } else if (type==WStype_CONNECTED) {
    Serial.printf("[WS] Client #%d connected\n", num);
  } else if (type==WStype_DISCONNECTED) {
    Serial.printf("[WS] Client #%d disconnected\n", num);
  }
}

// =============================================================================
//  Build and broadcast telemetry JSON
// =============================================================================
void sendTelemetry() {
  // {"r":roll,"p":pitch,"y":yaw,"ls":lAil,"rs":rAil,"es":elev,"rsv":rud,
  //  "c1":ch1,"c2":ch2,...,"vra":vra%,"mode":"NORMAL","crash":false,"fs":false,
  //  "swA":false,...}
  char buf[768];
  snprintf(buf, sizeof(buf),
    "{\"r\":%.1f,\"p\":%.1f,\"y\":%.1f,"
    "\"ls\":%d,\"rs\":%d,\"es\":%d,\"rsv\":%d,"
    "\"c1\":%d,\"c2\":%d,\"c3\":%d,\"c4\":%d,"
    "\"c5\":%d,\"c6\":%d,\"c7\":%d,\"c8\":%d,\"c9\":%d,"
    "\"vra\":%.0f,\"mode\":\"%s\","
    "\"crash\":%s,\"fs\":%s,"
    "\"swA\":%s,\"swB\":%s,\"swC\":%s,\"swD\":%s,"
    "\"ts\":%lu}",
    fc.t_roll, fc.t_pitch, fc.t_yaw,
    fc.t_ls, fc.t_rs, fc.t_es, fc.t_rv,
    fc.t_ch[0],fc.t_ch[1],fc.t_ch[2],fc.t_ch[3],
    fc.t_ch[4],fc.t_ch[5],fc.t_ch[6],fc.t_ch[7],fc.t_ch[8],
    fc.t_vra, fc.t_mode.c_str(),
    fc.t_crash?"true":"false", fc.t_fs?"true":"false",
    fc.t_swA?"true":"false", fc.t_swB?"true":"false",
    fc.t_swC?"true":"false", fc.t_swD?"true":"false",
    millis()
  );
  wsServer.broadcastTXT(buf);
}

static void initWatchdog() {
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t wdt_config = {};
  wdt_config.timeout_ms = WDT_SEC * 1000;
  wdt_config.idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1;
  wdt_config.trigger_panic = true;
  esp_task_wdt_init(&wdt_config);
#else
  esp_task_wdt_init(WDT_SEC, true);
#endif
  esp_task_wdt_add(NULL);
}

// =============================================================================
//  Setup
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("\n[AUTOSTABI] IBus Edition — Booting..."));

  // Watchdog
  initWatchdog();

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  if (!imu.begin()) {
    Serial.println(F("[IMU] Running without IMU (attitude = 0)"));
  }

  // IBus
  ibus.begin();

  // Flight controller
  fc.begin(&imu, &ibus);

  // WiFi AP
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.print(F("[WiFi] AP IP: "));
  Serial.println(WiFi.softAPIP());

  // HTTP — serve embedded GroundStation.html
  httpServer.on("/", HTTP_GET, []() {
    httpServer.send_P(200, "text/html", GS_HTML);
  });
  httpServer.onNotFound([]() {
    httpServer.send_P(200, "text/html", GS_HTML);
  });
  httpServer.begin();
  Serial.printf("[HTTP] Server on port %d\n", HTTP_PORT);

  // WebSocket
  wsServer.begin();
  wsServer.onEvent(wsEvent);
  Serial.printf("[WS]   Server on port %d\n", WS_PORT);

  Serial.println(F("[AUTOSTABI] Ready. Connect to WiFi AUTOSTABI / flysafe123"));
  Serial.println(F("[AUTOSTABI] Then open http://192.168.4.1"));

  lastImuMs = lastServoMs = lastWsMs = millis();
}

// =============================================================================
//  Main Loop  (non-blocking millis scheduling)
// =============================================================================
void loop() {
  uint32_t now = millis();

  // Feed watchdog every loop
  esp_task_wdt_reset();

  // IBus — continuous read
  ibus.update();

  // HTTP keep-alive
  httpServer.handleClient();

  // WebSocket keep-alive
  wsServer.loop();

  // IMU + Kalman + Flight Controller @ 200 Hz (every 5 ms)
  if (now - lastImuMs >= 5) {
    float dt = (now - lastImuMs) * 0.001f;
    lastImuMs = now;
    imu.update();
    if (!imu.healthy) {
      if (imuUnhealthySinceMs == 0) imuUnhealthySinceMs = now;
      else if (now - imuUnhealthySinceMs > 2000) {
        imu.begin();
        imuUnhealthySinceMs = imu.healthy ? 0 : now;
      }
    } else {
      imuUnhealthySinceMs = 0;
    }
    fc.computeControl(dt, now);
  }

  // Servo write @ 50 Hz (every 20 ms)
  if (now - lastServoMs >= 20) {
    lastServoMs = now;
    fc.applyServos();
  }

  // WebSocket telemetry @ 20 Hz (every 50 ms)
  if (now - lastWsMs >= 50) {
    lastWsMs = now;
    sendTelemetry();
  }
}
