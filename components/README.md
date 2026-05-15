# Components: Reusable Control Modules

This directory contains reusable, well-defined control modules that serve as building blocks for experiments and applications. Each component:
- Has its own Makefile and build rules
- Exports a versioned public API (C headers in `include/`)
- Can be tested independently
- Is linked by experiments that declare a dependency on it

## Component Structure

Each component folder contains:
- `src/` — Implementation files (.c)
- `include/` — Public headers (.h)
- `Makefile` — Component-local build rules
- `README.md` — Component documentation (purpose, API, usage)

## Components

### motor_control/
Soft motor acceleration and deceleration control.
- **Source**: Originally `rc_project_template/rc_motor_ramp.c`
- **Public API**: `rc_motor_ramp.h`
- **Used by**: `robot_server`, `shuffle_tester` experiments
- **Purpose**: Background thread ramp control with duty-cycle limits

### heading_control/
Gyroscope-based heading correction and magnetometer seeding.
- **Source**: Originally `rc_project_template/rc_gyro_heading.c`
- **Public API**: `rc_gyro_heading.h`
- **Used by**: `robot_server` experiment
- **Purpose**: Real-time gyro fusion with magnetometer updates for heading accuracy

## Build Integration

The master Makefile at `librobotcontrol/` orchestrates:
1. Library build (`library/`)
2. Component builds (this directory)
3. Experiment builds (`../experiments/`)
4. Service builds (`../services/`)

To add a new reusable component:
1. Create a folder under `components/` with name matching the module name
2. Implement `src/` and `include/` following existing patterns
3. Write a Makefile that compiles to `build/` and exports paths
4. Document usage in `README.md`
5. Update experiments' Makefiles to declare `BUILD_COMPONENTS += new_component_name`

## Future Extensibility

Planned components for hardware modularity:
- `battery_sensing/` — Battery monitoring abstraction
- `sensor_framework/` — Generic sensor driver template
- `servo_control/` — Servo positioning library
- `imu_fusion/` — Multi-axis IMU sensor fusion

Each would follow the same pattern and be consumed by corresponding experiments.
