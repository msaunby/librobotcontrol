This is meant to be a starting point for your own project.

For full instructions on how to use this template, see:

<http://strawsondesign.com/docs/librobotcontrol/docs/html/project_template.html>

Quick start in this directory:

- Build default template app:

	make

- Run default template app:

	make run

- Build and run ADC exploration preflight checker (battery charged and no DC jack power):

	make preflight

- Continuous edit-build-run loop for the preflight checker:

	make devloop

- Build and run battery endurance logger (CSV with durable flush on each row):

	make endurance-log

- Use a 30 second logging interval during long discharge tests:

	make endurance-log-watch

- Start logging in background so it continues after SSH disconnect:

	make endurance-start-bg

- Check logger status and latest rows:

	make endurance-status

- Stop background logger:

	make endurance-stop

- Build and run shuffle endurance test (motors reverse direction periodically while logging battery + accel):

	make shuffle-log

- Start shuffle endurance test in background (survives SSH/Wi-Fi disconnect):

	make shuffle-start-bg

  Note: this target uses --no-adc by default to avoid BeagleBone ADC/IIO stalls during long movement tests.
	It also uses conservative reversal-check defaults to reduce false stops: --reverse-verify-threshold 0.25 and --max-unverified-toggles 20.

- Check shuffle logger status and latest rows:

	make shuffle-status

- Stop shuffle logger:

	make shuffle-stop

Shuffle CSV includes accelerometer movement verification fields:

- accel_x_mean, accel_y_mean, accel_z_mean: mean acceleration over each interval
- accel_mag_mean: mean acceleration vector magnitude
- accel_mag_std: magnitude variation within each interval (higher means more motion/vibration)
- toggle_event: 1 when motor direction changed since previous sample
- reversal_verified: 1 if accel delta confirms reversal, 0 if unverified, -1 when no toggle event
- reversal_score: abs(delta accel_x_mean) + abs(delta accel_y_mean) across a toggle

Reversal verification controls:

- --reverse-verify-threshold V (default 0.80)
- --max-unverified-toggles N (default 3)

ADC mode control for shuffle test:

- --no-adc runs movement verification without battery/jack reads (recommended when ADC driver intermittently stalls)

Safety behavior:

- If reversals are not verified for N consecutive toggles, the program brakes motors and exits with a failsafe event.

Shuffle logger also writes a heartbeat/status file for post-mortem recovery:

- default file: shuffle_heartbeat.txt
- contains: last event, reason, elapsed time, motor direction, and commanded motor values
- updates on startup, direction toggles, running loop, and all stop/failsafe events

Heartbeat options:

- set custom heartbeat path: --heartbeat my_heartbeat.txt
- disable heartbeat writes: --no-heartbeat

Notes for endurance testing:

- Logging is local on the BeagleBone, so Wi-Fi dropouts do not stop capture.
- Each CSV row is flushed and synced to disk to reduce data loss if power fails.
- Default thresholds: explore-ready cell >= 3.95V, stop test at cell <= 3.30V.

---

Current project status (latest verified)

- Robot TCP control server is working for FORWARD, BACKWARD, TURN, STOP, HOME, STATUS.
- Motor ramping path is active and driver calls are succeeding (motor_m0_ret=0, motor_m1_ret=0 in STATUS).
- Gyro heading integration is active and responsive.
- Compass path is active and now tilt-compensated; compass_deg and raw mag fields change across samples.
- Magnetometer health during latest checks: compass_valid=1, mag_errors=0, mag_samples increasing.
- Calibration sequence script has been created and run successfully from the controlling laptop.

Latest useful test artifacts

- Calibration log file from latest full sequence:

	calibration_logs/calibration_run_20260512_152157.jsonl

- This run captured all three phases (A/B/C), had 100% STATUS parse success, and zero magnetometer read errors.

How to re-run key tests

1) Build and start server on BeagleBone

- from rc_project_template:

	make build-server
	make server-start-bg

2) Quick live status check from laptop

- run from controlling laptop:

	nc beaglebone.local 5000

- then send:

	STATUS
	FORWARD 0.30
	STATUS
	STOP
	STATUS

3) Full guided calibration sequence (recommended)

- from repository root on laptop:

	python3 scripts/run_calibration_sequence.py --auto-start --auto-stop

- output log path pattern:

	calibration_logs/calibration_run_YYYYMMDD_HHMMSS.jsonl

4) Validate battery and external power state before motion tests

- on BeagleBone in rc_project_template:

	make preflight

- expected for battery-only testing:

	cell >= 3.95V and DC Jack near 0.00V

Suggested restart workflow after a break

- Pull latest code in both repos.
- In rc_project_template, run: make build-server
- Run one quick STATUS/FORWARD/STOP sanity check.
- Run full calibration script once and compare new log metrics against the latest baseline above.
