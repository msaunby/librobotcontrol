# Battery Experiments

Characterize and validate the power subsystem through preflight checks and long-duration logging.

## Experiments

### Preflight Checker
**Source**: `rc_explore_preflight.c`

Pre-flight validation before endurance tests:
- ADC system initialization
- Battery cell voltages (3S LiPo: 3x series cells, typical 3.0-4.2V nominal)
- DC jack voltage (USB power presence check)

**Target**: `make preflight` or `make preflight-watch` (looping)

**Output**: PASS/FAIL for each subsystem to terminal

### Battery Endurance Logger
**Source**: `rc_battery_endurance_log.c`

Long-duration battery performance monitoring:
- Continuous voltage sampling (calibrated ADC read)
- Acceleration telemetry (IMU data to detect load spikes)
- CSV format with millisecond timestamps
- fsync on each row (durable to power loss)

**Targets**:
- `make build-endurance` — Compile only
- `make endurance-log` — Single run in foreground
- `make endurance-start-bg` — Start background process
- `make endurance-status` — Poll PID file for current state
- `make endurance-stop` — Graceful shutdown

**Output**: `battery_endurance.csv` with columns:
- `timestamp_ms`
- `battery_v` (cell voltage)
- `battery_a` (current draw, if current-sense available)
- `accel_x, accel_y, accel_z` (g units)
- `notes` (event/error descriptions)

**Typical Usage**:
```bash
cd librobotcontrol/experiments/battery
make endurance-start-bg
sleep 600  # 10 minutes of logging
make endurance-stop
cat battery_endurance.csv
```

## Baseline Metrics

Expected performance for healthy battery + power system:
- **Cell Voltage**: 3.8–4.1V under light load, 3.5–3.8V under 5A load
- **Voltage Sag**: <0.3V during motor acceleration (healthy cells)
- **Resting Voltage**: Returns to >3.8V within 30s of load shutdown
- **Temperature**: No significant rise during 1-hour test

## Troubleshooting

### ADC Initialization Fails
- Check ADC kernel module: `lsmod | grep gpio` (should see `am335x_adc`)
- Verify device node: `ls -la /sys/bus/iio/devices/iio:device0/`
- Run preflight: `make preflight`

### Low Voltage Readings
- Verify battery connection to BeagleBone XT60 connector
- Check for corroded contacts or loose JST connectors
- Calibrate ADC offset if consistently offset by constant value

### Background Logger Crashes
- Check `/proc/[pid]/stat` for CPU usage spike
- Verify SD card free space: `df -h /`
- Review system dmesg: `dmesg | tail -20` for OOM or I/O errors

## Migration Note

**Source Origin**: `librobotcontrol/rc_project_template/rc_battery_endurance_log.c` and `rc_explore_preflight.c`
**Migration Date**: May 2026
**Status**: Moved to dedicated battery experiment folder; Makefiles wired for component consumption
