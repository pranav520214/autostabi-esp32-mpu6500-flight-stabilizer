# WebSocket Telemetry

The firmware broadcasts JSON telemetry on WebSocket port `81`.

Default endpoint:

```text
ws://192.168.4.1:81
```

## Fields

| Field | Meaning |
|---|---|
| `r` | Roll angle in degrees |
| `p` | Pitch angle in degrees |
| `y` | Yaw rate in degrees/second |
| `ls` | Left aileron servo pulse in microseconds |
| `rs` | Right aileron servo pulse in microseconds |
| `es` | Elevator servo pulse in microseconds |
| `rsv` | Rudder servo pulse in microseconds |
| `c1`–`c9` | IBus channel values, usually 1000–2000 µs |
| `vra` | VRA sensitivity percentage |
| `mode` | Current displayed mode/status string |
| `crash` | Crash-prevention/recovery flag |
| `fs` | Failsafe flag |
| `swA`–`swD` | Switch state flags |
| `ts` | Device timestamp from `millis()` |

## Example

```json
{
  "r": 1.2,
  "p": -0.8,
  "y": 0.0,
  "ls": 1500,
  "rs": 1500,
  "es": 1500,
  "rsv": 1500,
  "c1": 1500,
  "c2": 1500,
  "c3": 1000,
  "c4": 1500,
  "c5": 1000,
  "c6": 1000,
  "c7": 1500,
  "c8": 1000,
  "c9": 1000,
  "vra": 50,
  "mode": "NORMAL",
  "crash": false,
  "fs": false,
  "swA": false,
  "swB": false,
  "swC": false,
  "swD": false,
  "ts": 123456
}
```
