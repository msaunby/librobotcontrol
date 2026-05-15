# Motor Control Component

Soft acceleration and deceleration control for motor duty-cycle ramping.

## Purpose

Provides background-thread motor control that smoothly ramps motor duty cycles up and down, preventing mechanical shock and EMI from rapid duty changes. Tracks left/right motor states independently.

## API

Public header: `include/rc_motor_ramp.h`

Key functions:
- `rc_motor_ramp_init()` — Initialize ramp system
- `rc_motor_ramp_set_duty()` — Request target duty cycle
- `rc_motor_ramp_stop()` — Halt all motors
- `rc_motor_ramp_cleanup()` — Stop background thread

## Usage

Experiments that need smooth motor control:
- `../experiments/robot_server/` — TCP-controlled navigation
- `../experiments/motors/shuffle_test/` — Endurance testing with duty sweeps

## Build

```bash
cd librobotcontrol/components/motor_control
make clean
make
```

Produces object files linked into applications.

## Migration Note

**Source Origin**: `librobotcontrol/rc_project_template/rc_motor_ramp.c` (v1.0.5)
**Migration Date**: May 2026
**Status**: Extracted, no functional changes to API
