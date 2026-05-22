# Hardware Notes

## Controller

- ESP32 development board
- Arduino ESP32 core

## Sensors and receiver

| Device | Connection |
|---|---|
| MPU6500 | I2C, SDA GPIO21, SCL GPIO22 |
| IBus receiver | Serial2 RX on GPIO16, 115200 baud |

## Servo outputs

| Surface | ESP32 pin | Pulse range |
|---|---:|---:|
| Left aileron | GPIO18 | 1000–2000 µs |
| Right aileron | GPIO19 | 1000–2000 µs |
| Rudder | GPIO23 | 1000–2000 µs |
| Elevator | GPIO5 | 1000–2000 µs |
| Flap/Aux | GPIO4 | 1000–2000 µs |

## Bench-test checklist

- Remove propellers before powering servos or motors.
- Confirm the IMU orientation and servo direction before any moving test.
- Confirm failsafe behavior with the transmitter off.
- Confirm the aircraft remains controllable in direct/manual mode.
