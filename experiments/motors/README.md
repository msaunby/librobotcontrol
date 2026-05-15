# Motor Experiments

Validate motor performance, endurance, and control characteristics through direction-change testing and duty-cycle characterization.

## Experiments

### Motor Endurance / Shuffle Test
**Source**: `rc_shuffle_endurance_log.c`

Extended motor operation with periodic direction reversals to stress motor windings and drive system:
- Configurable duty cycle (PWM percentage)
- Configurable reversal interval (toggle direction every N seconds)
- Accelerometer-based verification (detects stalled motor or mechanical jam)
- Heartbeat file for post-mortem recovery
- CSV logging of direction, duty, and accel telemetry

**Targets**:
- `make build-shuffle` — Compile only
- `make shuffle-log` — Single run in foreground
- `make shuffle-start-bg [DUTY=50] [INTERVAL=10]` — Start background with optional parameters
- `make shuffle-status` — Poll heartbeat and PID
- `make shuffle-stop` — Graceful shutdown

**Output Files**:
- `shuffle_endurance.csv` — Direction, duty, timestamp, accel_x/y/z, status
- `shuffle_heartbeat.txt` — Last event time, reason, motor state (for crash recovery)
- `shuffle_endurance.pid` — Background process ID

**Typical Usage**:
```bash
cd librobotcontrol/experiments/motors
make shuffle-start-bg DUTY=75 INTERVAL=5
sleep 1800  # 30 minutes
make shuffle-stop
cat shuffle_heartbeat.txt
tail -20 shuffle_endurance.csv
```

## Motor Profiles

### Nominal Performance (Healthy Motors)
- **Stall Current**: 5–6A per motor
- **Free-Run Current**: 0.3–0.5A per motor at 50% duty
- **Response Time**: Direction change within 50ms of toggle
- **Accel Spike**: <1.5g peak when starting from stall

### Known Failure Modes
- **Cogging**: Periodic torque ripple visible as oscillating accel every 50–200ms
- **Jam**: Accelerometer shows zero motion despite PWM output
- **Winding Short**: Duty <20% produces no torque; high current draw

## Baseline Test Procedure

**Equipment**: Charged 3S LiPo battery, motor-only rig (no attached load)

1. **Preflight**: `make preflight` in battery/ folder
2. **Light Load (30 min)**: `make shuffle-start-bg DUTY=50 INTERVAL=10`
3. **Heavy Load (30 min)**: `make shuffle-start-bg DUTY=80 INTERVAL=5`
4. **Cool-down**: Stop and let motor cool 15 minutes
5. **Analysis**: 
   - Plot `shuffle_endurance.csv` accel_x vs time
   - Verify direction changes match `INTERVAL` parameter
   - Confirm no stalls (accel stays >0.5g during motion)

## Troubleshooting

### Motor Doesn't Spin
- Verify PWM signal: `cat /sys/class/pwm/pwmchip0/pwm0/duty_cycle` (should change with duty param)
- Check motor connectors for corrosion or loose terminals
- Test with isolated motor supply (not BeagleBone +5V rail) if available

### Erratic Accel Readings
- Run accelerometer self-test: `librobotcontrol/examples/rc_test_imu` (if available in build)
- Power cycle BeagleBone; accelerometers sometimes lock up during boot
- Check I2C bus: `i2cdetect -y 2` (motor controllers should respond)

### Background Process Dies Unexpectedly
- Check dmesg: `dmesg | grep -i "motor\|shuffle"` for kernel-level faults
- Look for OOM: `dmesg | grep "Out of memory"`
- Verify disk space: `df -h /` (need >100MB for CSV logging)

## Future Enhancements

**Motor Characterization App** (planned):
- Sweep duty cycles from 10% to 100% in 5% steps
- Log speed (via encoder feedback if available) and current for each step
- Generate duty-to-torque curves
- Identify nonlinear regions and efficiency operating point

## Migration Note

**Source Origin**: `librobotcontrol/rc_project_template/rc_shuffle_endurance_log.c`
**Migration Date**: May 2026
**Status**: Moved to dedicated motor experiment folder; Makefiles wired for component consumption
