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
