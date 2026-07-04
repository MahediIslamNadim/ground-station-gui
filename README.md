# NexCore Ground Station

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue)]()
[![Platform](https://img.shields.io/badge/platform-ESP8266-orange)]()
[![PyPI](https://img.shields.io/badge/pip-install-green)]()

Professional ground station GUI for ESP8266-based drone flight controllers. Features real-time 3D attitude visualization, MAVLink telemetry, sensor calibration, and flight data logging.

> **Flight Controller Firmware**: [github.com/MahediIslamNadim/drone-flight-controller](https://github.com/MahediIslamNadim/drone-flight-controller)

## Features

- **3D Attitude Visualization** — Real-time aircraft orientation with pitch, roll, yaw
- **Sensor Calibration** — Step-by-step accelerometer and gyroscope calibration wizard
- **Live Graphing** — Real-time plots for accelerometer, gyroscope, and altitude data
- **MAVLink Telemetry** — Full MAVLink v1 protocol support for parameter read/write
- **Compass / Heading** — Directional display with heading indicator
- **Data Export** — Export logs to JSON, CSV, and MAVLink formats
- **Serial Auto-Detect** — Automatic port detection and connection management
- **Flight Logging** — Black box data recording for post-flight analysis

## Installation

### Via pip

```bash
pip install nexcore-ground-station
```

### From source

```bash
git clone https://github.com/MahediIslamNadim/ground-station-gui.git
cd ground-station-gui
pip install -r requirements.txt
```

## Usage

```bash
python drone_calibration.py
```

### Quick Start

1. Connect your ESP8266 flight controller via USB
2. Launch the ground station: `python drone_calibration.py`
3. Select the serial port from the dropdown and click **CONNECT**
4. Use **SCAN** to auto-detect the device
5. Navigate through tabs: 3D View, Graphs, Calibration, Compass

### Calibration

| Step | Action |
|------|--------|
| 1 | Place sensor on a level surface |
| 2 | Click **CALIBRATE ACCEL** |
| 3 | Wait for calibration to complete |
| 4 | Keep sensor perfectly still |
| 5 | Click **CALIBRATE GYRO** |
| 6 | Save profile via **Save** / **Export** |

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Ground Station GUI                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────┐ │
│  │ 3D View  │  │  Graphs  │  │ Compass  │  │  Logger  │ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬────┘ │
│       └──────────────┴─────────────┴──────────────┘      │
│                         │                                 │
│  ┌──────────────────────┴──────────────────────────┐     │
│  │              SerialConn / MAVLink                │     │
│  └──────────────────────┬──────────────────────────┘     │
├─────────────────────────┼────────────────────────────────┤
│              ESP8266 Flight Controller                    │
│              (Serial / MAVLink protocol)                  │
└─────────────────────────────────────────────────────────┘
```

## API

### GroundStation class

Main application window. Handles UI, serial communication, and data processing.

```python
from nexcore_ground_station import GroundStation

app = GroundStation()
app.mainloop()
```

### SerialConn class

Manages serial port connection to the flight controller.

```python
from nexcore_ground_station import SerialConn

conn = SerialConn(port="COM3", baud=115200)
conn.connect()
data = conn.read_sensor_data()
```

### MAVLink class

MAVLink v1 protocol encoder/decoder for telemetry.

```python
from nexcore_ground_station import MAVLink

mav = MAVLink()
packet = mav.encode_heartbeat()
mav.send(packet)
```

## Serial Protocol

| Command | Description |
|---------|-------------|
| `CALIBRATE` | Run full IMU calibration |
| `RESET` | Reset calibration offsets |
| `STATUS` | Display current sensor offsets |
| `ARM` | Arm motors |
| `DISARM` | Disarm motors |
| `HELP` | List all commands |

## Project Structure

```
├── drone_calibration.py       # Main GUI application (GroundStation, MAVLink, SerialConn)
├── test_ports.py              # Serial port diagnostic tool
├── requirements.txt           # Python dependencies
├── pyproject.toml             # Package configuration
├── CHANGELOG.md               # Version history
├── LICENSE                    # MIT License
└── README.md                  # This file
```

## Dependencies

- [pyserial](https://github.com/pyserial/pyserial) — Serial port communication
- [numpy](https://numpy.org) — Numerical computation
- [matplotlib](https://matplotlib.org) — Real-time data plotting

## Troubleshooting

**No serial port detected:**
- Install CH340/CP2102 USB-to-serial drivers
- Verify USB cable supports data transfer
- Check device manager for port assignment

**No sensor data:**
- Verify MPU6050 wiring (SDA/SCL)
- Confirm baud rate matches firmware (115200)
- Send `HELP` via terminal to test link

**Calibration fails:**
- Ensure level surface for accelerometer
- Keep completely still for gyroscope
- Wait for each step to reach 100%

## License

MIT License — see [LICENSE](LICENSE).

## Changelog

See [CHANGELOG.md](CHANGELOG.md) for version history.
