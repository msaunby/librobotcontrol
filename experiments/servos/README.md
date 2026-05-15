# Servo Experiments (Scaffolding)

Hardware validation for servo positioning, speed, and control characteristics.

## Planned Experiments

### Servo Sweep Test (Future)
**Purpose**: Characterize servo position response and linearity

**Typical Test**:
- Command servo to sweep from 0° to 180° and back in 1° increments
- Log commanded position and measured feedback (if available)
- Verify response within ±5° of command
- Measure response time (settling time to within 2° of target)

**Output**: CSV with timestamp, commanded_pos_deg, feedback_pos_deg, response_time_ms

### Servo Load Test (Future)
**Purpose**: Validate servo torque and load handling

**Typical Test**:
- Hold servo at fixed position with gradually increasing load
- Log current draw and position drift
- Determine stall current and holding torque characteristics

**Output**: Load vs current curve, holding torque threshold

### Servo Endurance Test (Future)
**Purpose**: Evaluate servo lifespan under extended duty

**Typical Test**:
- Continuous position sweeping over 24+ hours
- Log position command errors, current spikes, thermal events
- Detect early wear or gearbox damage

**Output**: Continuous telemetry with cumulative error analysis

## Current Status

This folder is scaffolding for future servo work. To create a servo experiment:

1. Copy this README.md as a template
2. Create `src/` and `include/` directories
3. Add your servo control source code
4. Create a `Makefile` following patterns in other experiment folders
5. Document baseline metrics and troubleshooting

## Servo Control via librobotcontrol

**Note**: BeagleBone Robotics Cape natively supports servo control via PRU (Programmable Real-time Unit).

Available APIs (in `librobotcontrol/library/include/rc/`):
- `servo.h` — High-level servo positioning
- `pwm.h` — Low-level PWM for servo signals (1–2ms pulse width)
- `encoder.h` — Feedback sensors (if position feedback is available)

Example:
```c
#include <robotcontrol.h>

// Initialize servo channel 1
rc_servo_init_ch(1);

// Set position (1000–2000µs pulse width, typically 0–180°)
rc_servo_set_us(1, 1500);  // Neutral position
rc_servo_set_us(1, 1000);  // Full reverse
rc_servo_set_us(1, 2000);  // Full forward

// Cleanup
rc_servo_cleanup();
```

## Build Template

Save as `Makefile` in your servo experiment folder:

```makefile
CC       = arm-linux-gnueabihf-gcc
CFLAGS   = -Wall -Wextra -O2 -I/usr/include/robotcontrol
LDFLAGS  = -lrobotcontrol -lm
MAIN     ?= my_servo_test.c
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

## Safety Notes

- Always power servos from a separate 5–6V supply, not BeagleBone +5V rail
- Limit PWM duty cycle: standard servos expect 1–2ms pulses at 50Hz
- Disable PWM during BeagleBone reboot to avoid random servo movements
- Test servo response at low speed before high-speed sweeps

## Migration Note

**Status**: Scaffolding created May 2026
**Next**: Implement servo sweep, load, and endurance experiments as needed

## References

- BeagleBone Robotics Cape PRU: `librobotcontrol/pru_firmware/`
- Servo datasheets: consult individual servo manufacturer specs
