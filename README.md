# AUTOSTABI — ESP32 Fixed-Wing Autostabilization

AUTOSTABI is an ESP32-based fixed-wing aircraft stabilization prototype with an MPU6500 IMU, IBus receiver input, servo output mixing, and a browser-based WebSocket ground station.

> Safety note: this is experimental RC/embedded firmware. Bench-test without propellers, verify servo directions, failsafe behavior, and local RC/airspace rules before any real-world use.

## What is included

```text
AUTOSTABI/
├── firmware/
│   └── Firmware/
│       └── Firmware.ino          # ESP32 Arduino sketch
├── ground-station/
│   └── GroundStation.html        # Standalone browser ground station
├── media/
│   └── demo.mp4                  # Demo video
├── docs/
│   ├── hardware.md
│   └── telemetry.md
├── .gitignore
└── README.md
```

## Features

- ESP32 Wi-Fi access point for local ground-station access.
- WebSocket telemetry on port `81`, broadcast about every 50 ms.
- HTTP ground-station page served from the ESP32 on port `80`.
- MPU6500 attitude sensing over I2C.
- IBus receiver support over `Serial2` at 115200 baud.
- Servo output for left aileron, right aileron, rudder, elevator, and flap/aux.
- Basic mode toggling, VRA sensitivity control, failsafe status, switch status, and attitude visualization.

## Hardware pin map

| Function | ESP32 pin |
|---|---:|
| MPU6500 SDA | GPIO21 |
| MPU6500 SCL | GPIO22 |
| IBus RX | GPIO16 / Serial2 |
| Left aileron servo | GPIO18 |
| Right aileron servo | GPIO19 |
| Rudder servo | GPIO23 |
| Elevator servo | GPIO5 |
| Flap/Aux servo | GPIO4 |

## Arduino requirements

Install these libraries in Arduino IDE Library Manager:

- `ESP32Servo` by madhephaestus, version `0.13.0` or newer
- `WebSockets` by Markus Sattler, version `2.4.1` or newer

Also install the ESP32 board package in Arduino IDE.

## Build and upload

1. Open `firmware/Firmware/Firmware.ino` in Arduino IDE.
2. Select your ESP32 board and port.
3. Install the required libraries listed above.
4. Compile and upload.
5. After boot, connect to the ESP32 AP:
   - SSID: `AUTOSTABI`
   - Default password in firmware: `flysafe123`
6. Open the ground station at `http://192.168.4.1/`.

## Ground station

The firmware serves an embedded HTML ground station automatically. A standalone version is also included at:

```text
ground-station/GroundStation.html
```

The standalone page connects to:

```text
ws://192.168.4.1:81
```

## Telemetry fields

The ESP32 broadcasts JSON telemetry over WebSocket. See [`docs/telemetry.md`](docs/telemetry.md) for the expected field meanings.

## Security note

The Wi-Fi access point password is currently hard-coded in `Firmware.ino` for easy testing. Before public release or field use, change the password and avoid reusing any personal credentials.

## Demo

See [`media/demo.mp4`](media/demo.mp4).

## License

No license has been added yet. Before publishing publicly, choose a license such as MIT, Apache-2.0, GPL-3.0, or keep the project all-rights-reserved.
