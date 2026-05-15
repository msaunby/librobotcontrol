# Heading Control Component

Real-time gyroscope-based heading correction with magnetometer seeding.

## Purpose

Fuses gyroscope and magnetometer data to maintain accurate robot heading over time. Gyro provides fast angular feedback; compass provides long-term drift correction.

## API

Public header: `include/rc_gyro_heading.h`

Key functions:
- `rc_gyro_heading_init()` — Initialize fusion filter
- `rc_gyro_heading_update()` — Update with latest IMU sample
- `rc_gyro_heading_reseed_compass()` — Correct drift using compass bearing
- `rc_gyro_heading_get()` — Read current heading estimate

## Usage

Experiments requiring stable heading maintenance:
- `../experiments/robot_server/` — Gyro-corrected autonomous navigation
- `../experiments/sensors/calibration/` — Heading verification (future)

## Build

```bash
cd librobotcontrol/components/heading_control
make clean
make
```

Produces object files linked into applications.

## Migration Note

**Source Origin**: `librobotcontrol/rc_project_template/rc_gyro_heading.c` (v1.0.5)
**Migration Date**: May 2026
**Status**: Extracted, no functional changes to API
