# Sensor Experiments (Scaffolding)

Hardware validation for individual sensors and sensor fusion algorithms.

## Planned Experiments

### Compass Stability Test (Future)
**Purpose**: Validate magnetometer accuracy, thermal stability, and calibration

**Typical Test**:
- Rotate robot 360° and log compass heading vs IMU gyro heading
- Verify heading error <5° after full rotation
- Log temperature vs heading drift over 1-hour static test

**Output**: CSV with timestamp, mag_heading, gyro_heading, temp_c, error_deg

### Gyro Drift Characterization (Future)
**Purpose**: Quantify angular velocity bias over temperature range and time

**Typical Test**:
- Leave robot stationary for 1 hour with gyro sampling
- Log raw angular rates (should be ~0 rad/s)
- Calculate bias as mean drift
- Repeat at different ambient temperatures

**Output**: Drift_rate_deg_per_hour vs temperature curve

### Multi-Axis IMU Fusion Validation (Future)
**Purpose**: Verify gyro/accel/mag fusion provides stable orientation estimate

**Typical Test**:
- Perform slow 360° rotation with IMU data collection
- Compare final heading to starting heading (should be <1° error)
- Log roll/pitch stability during stationary periods

**Output**: Fused quaternion time-series with error metrics

## Current Status

This folder is scaffolding for future sensor work. To create a sensor experiment:

1. Copy this README.md as a template
2. Create `src/` and `include/` directories
3. Add your sensor test source code
4. Create a `Makefile` following patterns in other experiment folders
5. Document baseline metrics and troubleshooting

## Sensor API Reference

Available via librobotcontrol headers (in `librobotcontrol/library/include/rc/`):
- `mpu.h` — 9-DOF IMU (accel, gyro, magnetometer)
- `adc.h` — Analog-to-digital converter (battery monitoring)
- `pwm.h` — PWM output (motor/servo control)
- `encoder.h` — Motor encoder feedback (if installed)

Example:
```c
#include <robotcontrol.h>

// Initialize MPU9250 IMU
rc_mpu_config_t mpu_conf = rc_mpu_default_config();
rc_mpu_initialize_i2c(&imu, mpu_conf);

// Read accelerometer
if (rc_mpu_read_accel(&imu) == 0) {
    printf("accel: x=%.2f y=%.2f z=%.2f g\n", 
        imu.accel[0], imu.accel[1], imu.accel[2]);
}
```

## Build Template

Save as `Makefile` in your sensor experiment folder:

```makefile
CC       = arm-linux-gnueabihf-gcc
CFLAGS   = -Wall -Wextra -O2 -I/usr/include/robotcontrol
LDFLAGS  = -lrobotcontrol -lm
MAIN     ?= my_sensor_test.c
TARGET   = $(basename $(MAIN))
SOURCES  = $(MAIN)

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

run: all
	./$(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: all run clean
```

Then run:
```bash
make clean && make && make run
```

## Migration Note

**Status**: Scaffolding created May 2026
**Next**: Implement compass, gyro, and fusion experiments as prioritized
