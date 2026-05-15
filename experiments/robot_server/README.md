# Robot Server Integration

TCP-based remote control with gyroscope-corrected heading maintenance.

## Overview

The robot server acts as an integration point for autonomous navigation:
- Accepts TCP commands on port 5000 (FORWARD/BACKWARD/TURN/STOP/BRAKE/HOME/STATUS/SET_*)
- Maintains heading estimate via gyro+compass fusion (from `heading_control` component)
- Smoothly ramps motor duty cycles (from `motor_control` component)
- Logs command history and IMU telemetry

## Architecture

**Components Consumed**:
- `../components/motor_control/` — Duty-cycle ramping
- `../components/heading_control/` — Gyro-based heading correction

**Source**: `rc_robot_server.c` (migrated from `rc_project_template/`)

## Protocol (Port 5000)

### Motion Commands

```
FORWARD [duty]       — Move forward at duty (0–100, default 50)
BACKWARD [duty]      — Move backward
TURN_LEFT [duty]     — In-place left rotation
TURN_RIGHT [duty]    — In-place right rotation
TURN_TO [heading_deg] — Rotate to absolute heading (uses gyro correction)
STOP                 — Halt smoothly (ramp off)
BRAKE                — Hard stop (no ramp)
HOME                 — Return to heading 0° (north)
```

### Status Commands

```
STATUS               — Request telemetry (heading_deg, duty_l, duty_r, accel_xyz)
SET_SPEED [0–100]    — Override global speed limit
SET_HEADING_GAIN [0.0–1.0] — Adjust gyro correction feedback gain
```

### Response Format

Status responses return JSON:
```json
{
  "heading_deg": 45.2,
  "duty_left": 50,
  "duty_right": 50,
  "accel_x": 0.1,
  "accel_y": 0.05,
  "accel_z": 9.8
}
```

## Building

```bash
cd librobotcontrol/experiments/robot_server
make clean
make
```

Compilation links against:
- `../../components/motor_control/src/rc_motor_ramp.c`
- `../../components/heading_control/src/rc_gyro_heading.c`
- System librobotcontrol library (`-lrobotcontrol`)

## Running

### Single-Run Foreground
```bash
./rc_robot_server
```

Listens on port 5000. Connect with:
```bash
telnet beaglebone.local 5000
FORWARD 50
STATUS
STOP
```

### Background Operation
```bash
make server-start-bg
make server-status
make server-stop
```

Writes PID to `robot_server.pid` and output to `robot_server.out`.

## Integration with Calibration Workflow

The calibration sequence (`../workflows/run_calibration_sequence.py`) uses the robot server during Phase C:
1. Starts server in background on BeagleBone
2. Sends FORWARD/BACKWARD/TURN commands from controller machine
3. Logs gyro+compass samples to infer motor-to-heading mapping
4. Computes heading correction gains
5. Stops server

Output: Calibration metrics saved to `../../calibration_logs/calibration_run_*.jsonl`

## Troubleshooting

### Port 5000 Already in Use
```bash
lsof -i :5000  # Find process
kill -9 [pid]
```

### Heading Drift During Long Operation
- Compass may saturate in high-EMI environment (motor noise)
- Try `SET_HEADING_GAIN 0.5` to reduce compass reliance
- Re-run Phase B calibration for updated magnetic declination

### Motors Unresponsive
- Verify battery voltage: `cat /sys/class/iio/iio:device0/in_voltage_scale` (should be ~3.8V)
- Check motor connectors for corrosion

## Output Files

- `robot_server.pid` — Process ID (for `make server-status`)
- `robot_server.out` — Command log and telemetry
- Console telemetry during foreground run

## Migration Note

**Source Origin**: `librobotcontrol/rc_project_template/rc_robot_server.c` + helper modules
**Migration Date**: May 2026
**Status**: Moved to dedicated folder; rewired to consume `motor_control` and `heading_control` components
**API Stability**: Protocol frozen (backward compatible with calibration scripts)
