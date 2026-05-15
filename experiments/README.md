# Experiments: Hardware Evaluation Workflows

This directory contains runnable applications for evaluating and validating hardware subsystems (battery, motors, sensors, servos) and integration workflows (calibration, testing).

## Structure

Each experiment folder is independent and may consume one or more reusable components from `../components/`.

### Battery Experiments
**Folder**: `battery/`

Characterize power subsystem:
- `rc_explore_preflight.c` — Pre-flight checks (ADC, jack, battery voltage)
- `rc_battery_endurance_log.c` — Long-duration battery voltage and load logging

**Output**: CSV files with voltage/current/accel telemetry. Identifies degradation, anomalies.

### Motor Experiments
**Folder**: `motors/`

Validate motor characteristics:
- `rc_shuffle_endurance_log.c` — Direction-change endurance with accelerometer verification
- *(Future)* `rc_motor_characterization.c` — Duty-to-speed mapping at various loads

**Output**: CSV logs with motor state, direction, accelerometer data. Verifies consistent performance.

### Sensor Experiments
**Folder**: `sensors/`

Validate individual sensors and fusion:
- *(Scaffolding)* Compass stability testing
- *(Future)* Gyro drift characterization
- *(Future)* Multi-axis IMU fusion validation

**Output**: Raw sensor data, drift estimates, error statistics.

### Servo Experiments
**Folder**: `servos/`

Evaluate servo performance and control:
- *(Scaffolding)* Placeholder for servo sweep and feedback experiments
- *(Future)* Position linearity, speed response, load handling

**Output**: Position tracking data, response timing.

### Robot Server
**Folder**: `robot_server/`

Integration: TCP-based remote robot control with heading correction.
- Consumes: `motor_control`, `heading_control` components
- Protocol: TCP on port 5000 with FORWARD/BACKWARD/TURN/STOP/HOME/STATUS commands
- Replaces old `rc_project_template/rc_robot_server.c`

**Output**: Command log, status updates.

### Workflows
**Folder**: `workflows/`

Orchestration scripts and calibration sequences:
- `run_calibration_sequence.py` — Multi-phase compass and heading calibration
- *(Future)* `run_endurance_suite.py` — Parametric battery/motor sweep
- `test_protocols.json` — Protocol schema and baseline metrics (future)

## Running Experiments

Each experiment folder has a local Makefile. Example:

```bash
cd librobotcontrol/experiments/battery
make preflight        # Single-run preflight check
make endurance-bg     # Start background battery logger
make endurance-status # Poll logger status
make endurance-stop   # Stop and finalize log
```

See each experiment's README.md for detailed instructions.

## Log Outputs

All logs are written to the experiment's working directory:
- `battery/*.csv` — Battery monitoring data
- `motors/*.csv` — Motor characterization data
- `sensors/*.log` — Raw sensor data
- `workflows/*.jsonl` — Calibration event stream

## Adding New Experiments

1. Create a folder under `experiments/` matching the hardware subsystem
2. Add source files (.c), headers (.h), and a Makefile
3. In Makefile, declare `BUILD_COMPONENTS` for any reused modules
4. Write a README.md explaining purpose, usage, and output interpretation
5. Test: `make clean && make && make run`

## Architecture Notes

- **No hardcoded paths**: Experiments use relative paths for outputs; portable across systems
- **Component isolation**: Each experiment imports only the components it needs (declared in Makefile)
- **Backward compatibility**: Old calibration scripts updated to new remote directory paths
- **Extensibility**: Adding new sensors/servos means adding a new folder, not modifying existing apps

## Migration Status

**Phase 1 (May 2026)**: Folder structure and scaffolding created
- Components extracted and isolated
- Initial experiments migrated from rc_project_template
- Sensor and servo scaffolding created with README placeholders

**Phase 2**: Build system wired
**Phase 3**: Source migration and rewiring
**Phase 4**: Script and documentation updates
**Phase 5**: Validation and cleanup
