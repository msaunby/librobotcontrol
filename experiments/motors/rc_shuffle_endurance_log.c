/**
 * @file rc_shuffle_endurance_log.c
 *
 * Endurance test with back-and-forth motor shuffling plus battery and
 * accelerometer logging. Each row is flushed and synced to disk to reduce
 * data loss if power fails.
 */

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdbool.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <rc/adc.h>
#include <rc/motor.h>
#include <rc/mpu.h>
#include <rc/start_stop.h>
#include <rc/time.h>

#define I2C_BUS 2

#define DEFAULT_LOG_PATH "shuffle_endurance.csv"
#define DEFAULT_INTERVAL_SEC 5
#define DEFAULT_SAMPLES 20
#define DEFAULT_DISCONNECT_V 1.0
#define DEFAULT_MIN_CELL_V 3.95
#define DEFAULT_STOP_CELL_V 3.30
#define DEFAULT_MAX_JACK_V 1.0
#define DEFAULT_DUTY 0.35
#define DEFAULT_SHUFFLE_SEC 4
#define DEFAULT_MOTOR_CH 0
#define DEFAULT_HEARTBEAT_PATH "shuffle_heartbeat.txt"
#define DEFAULT_WATCHDOG_SEC 12
#define DEFAULT_REVERSE_VERIFY_THRESHOLD 0.80
#define DEFAULT_MAX_UNVERIFIED_TOGGLES 3
#define ADC_READ_RETRIES 10
#define ADC_RETRY_US 20000
#define MPU_READ_RETRIES 5
#define MPU_RETRY_US 10000
#define MOTOR_CH_M0 1
#define MOTOR_CH_M1 2

struct config_t {
	const char* log_path;
	int interval_sec;
	int samples;
	double disconnect_v;
	double min_cell_v;
	double stop_cell_v;
	double max_jack_v;
	double duty;
	int shuffle_sec;
	int motor_ch;
	int watchdog_sec;
	double reverse_verify_threshold;
	int max_unverified_toggles;
	const char* heartbeat_path;
	bool heartbeat_enabled;
	bool use_adc;
	bool require_no_external;
};

static void print_usage(const char* prog)
{
	printf("Usage: %s [options]\n", prog);
	printf("Options:\n");
	printf("  --log PATH           Output CSV path (default %s)\n", DEFAULT_LOG_PATH);
	printf("  --interval-sec N     Log interval in seconds (default %d)\n", DEFAULT_INTERVAL_SEC);
	printf("  --samples N          Sensor samples per row (default %d)\n", DEFAULT_SAMPLES);
	printf("  --duty D             Motor duty cycle magnitude 0..1 (default %.2f)\n", DEFAULT_DUTY);
	printf("  --shuffle-sec N      Direction toggle period in seconds (default %d)\n", DEFAULT_SHUFFLE_SEC);
	printf("  --motor-ch N         Motor channel 0-4, 0 means M0+M1 pair (default %d)\n", DEFAULT_MOTOR_CH);
	printf("  --watchdog-sec N     Brake if no successful direction update in N seconds (default %d)\n", DEFAULT_WATCHDOG_SEC);
	printf("  --reverse-verify-threshold V  Min accel delta score to verify reversal (default %.2f)\n", DEFAULT_REVERSE_VERIFY_THRESHOLD);
	printf("  --max-unverified-toggles N    Stop after N unverified toggles (default %d)\n", DEFAULT_MAX_UNVERIFIED_TOGGLES);
	printf("  --heartbeat PATH     Heartbeat/status file path (default %s)\n", DEFAULT_HEARTBEAT_PATH);
	printf("  --no-heartbeat       Disable heartbeat/status file updates\n");
	printf("  --no-adc             Disable ADC battery/jack reads (movement verification mode)\n");
	printf("  --cell-min V         Exploration threshold per cell (default %.2fV)\n", DEFAULT_MIN_CELL_V);
	printf("  --cell-stop V        Stop when cell <= V (default %.2fV)\n", DEFAULT_STOP_CELL_V);
	printf("  --jack-max V         Max jack voltage considered unplugged (default %.2fV)\n", DEFAULT_MAX_JACK_V);
	printf("  --disconnect-v V     Below this voltage, battery considered disconnected (default %.2fV)\n", DEFAULT_DISCONNECT_V);
	printf("  --allow-external     Continue even if external power is detected\n");
	printf("  --help               Show this help\n");
}

static int parse_int_arg(const char* flag, const char* val, int* out, int min_v, int max_v)
{
	char* end = NULL;
	long n = strtol(val, &end, 10);
	if(end == val || *end != '\0' || n < min_v || n > max_v){
		fprintf(stderr, "ERROR: invalid value for %s: %s\n", flag, val);
		return -1;
	}
	*out = (int)n;
	return 0;
}

static int parse_double_arg(const char* flag, const char* val, double* out)
{
	char* end = NULL;
	double d = strtod(val, &end);
	if(end == val || *end != '\0'){
		fprintf(stderr, "ERROR: invalid value for %s: %s\n", flag, val);
		return -1;
	}
	*out = d;
	return 0;
}

static int parse_args(int argc, char* argv[], struct config_t* cfg)
{
	for(int i = 1; i < argc; i++){
		if(strcmp(argv[i], "--log") == 0){
			if(i + 1 >= argc){
				fprintf(stderr, "ERROR: --log requires a value\n");
				return -1;
			}
			cfg->log_path = argv[++i];
		}
		else if(strcmp(argv[i], "--interval-sec") == 0){
			if(i + 1 >= argc || parse_int_arg("--interval-sec", argv[++i], &cfg->interval_sec, 1, 3600) < 0) return -1;
		}
		else if(strcmp(argv[i], "--samples") == 0){
			if(i + 1 >= argc || parse_int_arg("--samples", argv[++i], &cfg->samples, 1, 1000) < 0) return -1;
		}
		else if(strcmp(argv[i], "--duty") == 0){
			if(i + 1 >= argc || parse_double_arg("--duty", argv[++i], &cfg->duty) < 0) return -1;
		}
		else if(strcmp(argv[i], "--shuffle-sec") == 0){
			if(i + 1 >= argc || parse_int_arg("--shuffle-sec", argv[++i], &cfg->shuffle_sec, 1, 3600) < 0) return -1;
		}
		else if(strcmp(argv[i], "--motor-ch") == 0){
			if(i + 1 >= argc || parse_int_arg("--motor-ch", argv[++i], &cfg->motor_ch, 0, 4) < 0) return -1;
		}
		else if(strcmp(argv[i], "--watchdog-sec") == 0){
			if(i + 1 >= argc || parse_int_arg("--watchdog-sec", argv[++i], &cfg->watchdog_sec, 1, 3600) < 0) return -1;
		}
		else if(strcmp(argv[i], "--reverse-verify-threshold") == 0){
			if(i + 1 >= argc || parse_double_arg("--reverse-verify-threshold", argv[++i], &cfg->reverse_verify_threshold) < 0) return -1;
		}
		else if(strcmp(argv[i], "--max-unverified-toggles") == 0){
			if(i + 1 >= argc || parse_int_arg("--max-unverified-toggles", argv[++i], &cfg->max_unverified_toggles, 1, 100) < 0) return -1;
		}
		else if(strcmp(argv[i], "--heartbeat") == 0){
			if(i + 1 >= argc){
				fprintf(stderr, "ERROR: --heartbeat requires a value\n");
				return -1;
			}
			cfg->heartbeat_path = argv[++i];
		}
		else if(strcmp(argv[i], "--no-heartbeat") == 0){
			cfg->heartbeat_enabled = false;
		}
		else if(strcmp(argv[i], "--no-adc") == 0){
			cfg->use_adc = false;
		}
		else if(strcmp(argv[i], "--cell-min") == 0){
			if(i + 1 >= argc || parse_double_arg("--cell-min", argv[++i], &cfg->min_cell_v) < 0) return -1;
		}
		else if(strcmp(argv[i], "--cell-stop") == 0){
			if(i + 1 >= argc || parse_double_arg("--cell-stop", argv[++i], &cfg->stop_cell_v) < 0) return -1;
		}
		else if(strcmp(argv[i], "--jack-max") == 0){
			if(i + 1 >= argc || parse_double_arg("--jack-max", argv[++i], &cfg->max_jack_v) < 0) return -1;
		}
		else if(strcmp(argv[i], "--disconnect-v") == 0){
			if(i + 1 >= argc || parse_double_arg("--disconnect-v", argv[++i], &cfg->disconnect_v) < 0) return -1;
		}
		else if(strcmp(argv[i], "--allow-external") == 0){
			cfg->require_no_external = false;
		}
		else if(strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0){
			print_usage(argv[0]);
			return 1;
		}
		else{
			fprintf(stderr, "ERROR: unknown argument: %s\n", argv[i]);
			return -1;
		}
	}

	if(cfg->duty < 0.0 || cfg->duty > 1.0){
		fprintf(stderr, "ERROR: --duty must be in [0,1]\n");
		return -1;
	}
	if(cfg->reverse_verify_threshold < 0.0){
		fprintf(stderr, "ERROR: --reverse-verify-threshold must be >= 0\n");
		return -1;
	}
	if(cfg->stop_cell_v > cfg->min_cell_v){
		fprintf(stderr, "ERROR: --cell-stop should be <= --cell-min\n");
		return -1;
	}
	return 0;
}

static int open_log_file(const char* path, FILE** out_fp, int* out_fd)
{
	int fd = open(path, O_WRONLY | O_CREAT | O_APPEND, 0644);
	if(fd < 0){
		perror("ERROR: open log file");
		return -1;
	}

	struct stat st;
	if(fstat(fd, &st) < 0){
		perror("ERROR: fstat log file");
		close(fd);
		return -1;
	}

	FILE* fp = fdopen(fd, "a");
	if(fp == NULL){
		perror("ERROR: fdopen log file");
		close(fd);
		return -1;
	}

	if(st.st_size == 0){
		fprintf(fp,
			"epoch_s,iso8601,elapsed_s,pack_v,cell_v,dc_jack_v,battery_present,external_power,above_explore_threshold,status,m0_cmd,m1_cmd,motor_dir,accel_x_mean,accel_y_mean,accel_z_mean,accel_mag_mean,accel_mag_std,toggle_event,reversal_verified,reversal_score\n");
		fflush(fp);
		fdatasync(fd);
	}

	*out_fp = fp;
	*out_fd = fd;
	return 0;
}

static int sleep_until(struct timespec* ts, int sec)
{
	ts->tv_sec += sec;
	while(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts, NULL) == -1){
		if(errno == EINTR) continue;
		perror("ERROR: clock_nanosleep");
		return -1;
	}
	return 0;
}

static void set_motor_command(int motor_ch, double base_cmd, double* m0_cmd, double* m1_cmd)
{
	if(motor_ch == 0){
		// In dual-motor mode, reverse M1 polarity so both sides drive robot-forward together.
		*m0_cmd = base_cmd;
		*m1_cmd = -base_cmd;
		rc_motor_set(MOTOR_CH_M0, *m0_cmd);
		rc_motor_set(MOTOR_CH_M1, *m1_cmd);
	}
	else{
		*m0_cmd = base_cmd;
		*m1_cmd = 0.0;
		rc_motor_set(motor_ch, base_cmd);
	}
}

typedef struct {
	pthread_mutex_t lock;
	int running;
	int motor_ch;
	double duty;
	int shuffle_sec;
	int watchdog_sec;
	const char* heartbeat_path;
	bool heartbeat_enabled;
	int direction;
	double m0_cmd;
	double m1_cmd;
	struct timespec last_main_kick;
} motor_thread_state_t;

static int write_heartbeat(const char* path,
				   const char* event,
				   const char* reason,
				   double elapsed_s,
				   int direction,
				   double m0_cmd,
				   double m1_cmd);

static double monotonic_delta_s(struct timespec newer, struct timespec older)
{
	double s = (double)(newer.tv_sec - older.tv_sec);
	s += (double)(newer.tv_nsec - older.tv_nsec) / 1e9;
	return s;
}

static void* motor_thread_func(void* arg)
{
	motor_thread_state_t* s = (motor_thread_state_t*)arg;
	struct timespec next;
	clock_gettime(CLOCK_MONOTONIC, &next);

	while(1){
		int running;
		int direction;
		double duty;
		int motor_ch;
		int watchdog_sec;
		const char* heartbeat_path;
		bool heartbeat_enabled;
		struct timespec last_main_kick;

		pthread_mutex_lock(&s->lock);
		running = s->running;
		direction = s->direction;
		duty = s->duty;
		motor_ch = s->motor_ch;
		watchdog_sec = s->watchdog_sec;
		heartbeat_path = s->heartbeat_path;
		heartbeat_enabled = s->heartbeat_enabled;
		last_main_kick = s->last_main_kick;
		pthread_mutex_unlock(&s->lock);

		if(!running) break;

		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		if(monotonic_delta_s(now, last_main_kick) > (double)watchdog_sec){
			fprintf(stderr, "FAILSAFE: control heartbeat expired, braking motors\n");
			if(heartbeat_enabled){
				write_heartbeat(heartbeat_path,
					"FAILSAFE_STOP",
					"control heartbeat expired",
					0.0,
					direction,
					s->m0_cmd,
					s->m1_cmd);
			}
			rc_motor_brake(motor_ch);
			kill(getpid(), SIGTERM);
			rc_usleep(100000);
			_exit(2);
		}

		double m0 = 0.0;
		double m1 = 0.0;
		double cmd = duty * (double)direction;
		set_motor_command(motor_ch, cmd, &m0, &m1);

		pthread_mutex_lock(&s->lock);
		s->m0_cmd = m0;
		s->m1_cmd = m1;
		s->direction = -s->direction;
		pthread_mutex_unlock(&s->lock);

		next.tv_sec += s->shuffle_sec;
		while(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL) == -1){
			if(errno == EINTR) continue;
			break;
		}
	}

	return NULL;
}

static void current_iso8601(char* out, size_t out_len, time_t* epoch_out)
{
	time_t epoch = time(NULL);
	struct tm tmbuf;
	gmtime_r(&epoch, &tmbuf);
	strftime(out, out_len, "%Y-%m-%dT%H:%M:%SZ", &tmbuf);
	if(epoch_out != NULL) *epoch_out = epoch;
}

static int write_heartbeat(const char* path,
				   const char* event,
				   const char* reason,
				   double elapsed_s,
				   int direction,
				   double m0_cmd,
				   double m1_cmd)
{
	char iso[32] = {0};
	time_t epoch = 0;
	current_iso8601(iso, sizeof(iso), &epoch);

	int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if(fd < 0){
		perror("WARN: heartbeat open failed");
		return -1;
	}

	dprintf(fd,
		"epoch_s=%ld\n"
		"iso8601=%s\n"
		"event=%s\n"
		"reason=%s\n"
		"elapsed_s=%.3f\n"
		"motor_dir=%d\n"
		"m0_cmd=%.3f\n"
		"m1_cmd=%.3f\n",
		(long)epoch,
		iso,
		event,
		reason,
		elapsed_s,
		direction,
		m0_cmd,
		m1_cmd);

	if(fsync(fd) < 0){
		perror("WARN: heartbeat fsync failed");
	}
	close(fd);
	return 0;
}

static void emergency_brake_and_stop(int motor_ch,
				     const char* reason,
				     const char* heartbeat_path,
				     bool heartbeat_enabled,
				     double elapsed_s,
				     int direction,
				     double m0_cmd,
				     double m1_cmd)
{
	fprintf(stderr, "FAILSAFE: %s\n", reason);
	if(heartbeat_enabled){
		write_heartbeat(heartbeat_path, "FAILSAFE_STOP", reason, elapsed_s, direction, m0_cmd, m1_cmd);
	}
	rc_motor_brake(motor_ch);
	rc_set_state(EXITING);
}

int main(int argc, char* argv[])
{
	struct config_t cfg = {
		.log_path = DEFAULT_LOG_PATH,
		.interval_sec = DEFAULT_INTERVAL_SEC,
		.samples = DEFAULT_SAMPLES,
		.disconnect_v = DEFAULT_DISCONNECT_V,
		.min_cell_v = DEFAULT_MIN_CELL_V,
		.stop_cell_v = DEFAULT_STOP_CELL_V,
		.max_jack_v = DEFAULT_MAX_JACK_V,
		.duty = DEFAULT_DUTY,
		.shuffle_sec = DEFAULT_SHUFFLE_SEC,
		.motor_ch = DEFAULT_MOTOR_CH,
		.watchdog_sec = DEFAULT_WATCHDOG_SEC,
		.reverse_verify_threshold = DEFAULT_REVERSE_VERIFY_THRESHOLD,
		.max_unverified_toggles = DEFAULT_MAX_UNVERIFIED_TOGGLES,
		.heartbeat_path = DEFAULT_HEARTBEAT_PATH,
		.heartbeat_enabled = true,
		.use_adc = true,
		.require_no_external = true,
	};

	int parse = parse_args(argc, argv, &cfg);
	if(parse == 1) return 0;
	if(parse < 0){
		print_usage(argv[0]);
		return 1;
	}

	if(rc_enable_signal_handler() == -1){
		fprintf(stderr, "ERROR: failed to start signal handler\n");
		return 1;
	}
	if(cfg.use_adc){
		if(rc_adc_init() == -1){
			fprintf(stderr, "ERROR: failed to initialize ADC\n");
			return 1;
		}
	}

	rc_mpu_data_t mpu_data;
	rc_mpu_config_t mpu_conf = rc_mpu_default_config();
	mpu_conf.i2c_bus = I2C_BUS;
	mpu_conf.show_warnings = 0;
	if(rc_mpu_initialize(&mpu_data, mpu_conf) < 0){
		fprintf(stderr, "ERROR: failed to initialize MPU\n");
		if(cfg.use_adc) rc_adc_cleanup();
		return 1;
	}

	if(rc_motor_init() < 0){
		fprintf(stderr, "ERROR: failed to initialize motors\n");
		rc_mpu_power_off();
		if(cfg.use_adc) rc_adc_cleanup();
		return 1;
	}

	FILE* fp = NULL;
	int fd = -1;
	if(open_log_file(cfg.log_path, &fp, &fd) < 0){
		rc_motor_cleanup();
		rc_mpu_power_off();
		if(cfg.use_adc) rc_adc_cleanup();
		return 1;
	}

	printf("Shuffle endurance log: %s\n", cfg.log_path);
	printf("motor_ch=%d duty=%.2f shuffle_sec=%d interval_sec=%d samples=%d\n",
		cfg.motor_ch, cfg.duty, cfg.shuffle_sec, cfg.interval_sec, cfg.samples);
	printf("watchdog_sec=%d\n", cfg.watchdog_sec);
	printf("reverse_verify_threshold=%.2f max_unverified_toggles=%d\n",
		cfg.reverse_verify_threshold, cfg.max_unverified_toggles);
	printf("use_adc=%d\n", cfg.use_adc ? 1 : 0);
	if(cfg.heartbeat_enabled){
		printf("heartbeat=%s\n", cfg.heartbeat_path);
	}
	printf("Thresholds: cell_min=%.2fV cell_stop=%.2fV jack_max=%.2fV\n",
		cfg.min_cell_v, cfg.stop_cell_v, cfg.max_jack_v);

	rc_set_state(RUNNING);
	motor_thread_state_t motor_state;
	memset(&motor_state, 0, sizeof(motor_state));
	pthread_mutex_init(&motor_state.lock, NULL);
	motor_state.running = 1;
	motor_state.motor_ch = cfg.motor_ch;
	motor_state.duty = cfg.duty;
	motor_state.shuffle_sec = cfg.shuffle_sec;
	motor_state.watchdog_sec = cfg.watchdog_sec;
	motor_state.heartbeat_path = cfg.heartbeat_path;
	motor_state.heartbeat_enabled = cfg.heartbeat_enabled;
	motor_state.direction = 1;
	clock_gettime(CLOCK_MONOTONIC, &motor_state.last_main_kick);

	pthread_t motor_thread;
	if(pthread_create(&motor_thread, NULL, motor_thread_func, &motor_state) != 0){
		fprintf(stderr, "ERROR: failed to start motor control thread\n");
		rc_motor_cleanup();
		rc_mpu_power_off();
		if(cfg.use_adc) rc_adc_cleanup();
		fclose(fp);
		return 1;
	}

	int direction = 1;
	double m0_cmd = 0.0;
	double m1_cmd = 0.0;
	int prev_logged_direction = 0;
	double prev_ax_mean = 0.0;
	double prev_ay_mean = 0.0;
	int have_prev_accel = 0;
	int consecutive_unverified_toggles = 0;
	if(cfg.heartbeat_enabled){
		write_heartbeat(cfg.heartbeat_path, "START", "startup", 0.0, direction, 0.0, 0.0);
	}

	struct timespec t0;
	clock_gettime(CLOCK_MONOTONIC, &t0);
	struct timespec wake = t0;

	while(rc_get_state() != EXITING){
		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		pthread_mutex_lock(&motor_state.lock);
		motor_state.last_main_kick = now;
		pthread_mutex_unlock(&motor_state.lock);
		double elapsed_s = (double)(now.tv_sec - t0.tv_sec) +
			((double)(now.tv_nsec - t0.tv_nsec) / 1e9);

		pthread_mutex_lock(&motor_state.lock);
		direction = motor_state.direction;
		m0_cmd = motor_state.m0_cmd;
		m1_cmd = motor_state.m1_cmd;
		pthread_mutex_unlock(&motor_state.lock);

		double pack_sum = 0.0;
		double jack_sum = 0.0;
		double ax_sum = 0.0;
		double ay_sum = 0.0;
		double az_sum = 0.0;
		double amag_sum = 0.0;
		double amag2_sum = 0.0;

		for(int i = 0; i < cfg.samples; i++){
			clock_gettime(CLOCK_MONOTONIC, &now);
			pthread_mutex_lock(&motor_state.lock);
			motor_state.last_main_kick = now;
			pthread_mutex_unlock(&motor_state.lock);
			double p = -1.0;
			double j = -1.0;
			if(cfg.use_adc){
				int adc_ok = 0;
				for(int a = 0; a < ADC_READ_RETRIES; a++){
					p = rc_adc_batt();
					j = rc_adc_dc_jack();
					if(p >= 0.0 && j >= 0.0){
						adc_ok = 1;
						break;
					}
					rc_usleep(ADC_RETRY_US);
				}
				if(rc_get_state() == EXITING) break;
				if(!adc_ok){
					emergency_brake_and_stop(cfg.motor_ch,
						"ADC read failed after retries",
						cfg.heartbeat_path,
						cfg.heartbeat_enabled,
						elapsed_s,
						direction,
						m0_cmd,
						m1_cmd);
					break;
				}
			}
			else{
				p = 0.0;
				j = 0.0;
			}

			int mpu_ok = 0;
			for(int a = 0; a < MPU_READ_RETRIES; a++){
				if(rc_mpu_read_accel(&mpu_data) == 0){
					mpu_ok = 1;
					break;
				}
				rc_usleep(MPU_RETRY_US);
			}
			if(!mpu_ok){
				emergency_brake_and_stop(cfg.motor_ch,
					"MPU accel read failed after retries",
					cfg.heartbeat_path,
					cfg.heartbeat_enabled,
					elapsed_s,
					direction,
					m0_cmd,
					m1_cmd);
				break;
			}
			double ax = mpu_data.accel[0];
			double ay = mpu_data.accel[1];
			double az = mpu_data.accel[2];
			double amag = sqrt(ax * ax + ay * ay + az * az);

			pack_sum += p;
			jack_sum += j;
			ax_sum += ax;
			ay_sum += ay;
			az_sum += az;
			amag_sum += amag;
			amag2_sum += amag * amag;
			rc_usleep(10000);
		}
		if(rc_get_state() == EXITING) break;

		pthread_mutex_lock(&motor_state.lock);
		direction = motor_state.direction;
		m0_cmd = motor_state.m0_cmd;
		m1_cmd = motor_state.m1_cmd;
		pthread_mutex_unlock(&motor_state.lock);

		double pack_v = pack_sum / cfg.samples;
		double jack_v = jack_sum / cfg.samples;
		double ax_mean = ax_sum / cfg.samples;
		double ay_mean = ay_sum / cfg.samples;
		double az_mean = az_sum / cfg.samples;
		double amag_mean = amag_sum / cfg.samples;
		double amag_var = (amag2_sum / cfg.samples) - (amag_mean * amag_mean);
		double amag_std = amag_var > 0.0 ? sqrt(amag_var) : 0.0;
		int toggle_event = (have_prev_accel && direction != prev_logged_direction) ? 1 : 0;
		double reversal_score = 0.0;
		int reversal_verified = -1;
		if(toggle_event){
			reversal_score = fabs(ax_mean - prev_ax_mean) + fabs(ay_mean - prev_ay_mean);
			reversal_verified = (reversal_score >= cfg.reverse_verify_threshold) ? 1 : 0;
			if(reversal_verified){
				consecutive_unverified_toggles = 0;
			}
			else{
				consecutive_unverified_toggles++;
			}
		}
		prev_logged_direction = direction;
		prev_ax_mean = ax_mean;
		prev_ay_mean = ay_mean;
		have_prev_accel = 1;

		bool battery_present = pack_v >= cfg.disconnect_v;
		double cell_v = battery_present ? pack_v / 2.0 : 0.0;
		bool external_power = jack_v >= cfg.max_jack_v;
		bool above_explore = battery_present && (cell_v >= cfg.min_cell_v);
		if(!cfg.use_adc){
			battery_present = 1;
			cell_v = cfg.min_cell_v;
			external_power = 0;
			above_explore = 1;
		}

		const char* status = "RUNNING";
		if(!battery_present){
			status = "NO_BATTERY";
		}
		else if(external_power && cfg.require_no_external){
			status = "EXTERNAL_POWER";
		}
		else if(cell_v <= cfg.stop_cell_v){
			status = "STOP_LOW_BATTERY";
		}

		time_t epoch = 0;
		char iso[32] = {0};
		current_iso8601(iso, sizeof(iso), &epoch);

		fprintf(fp,
			"%ld,%s,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%s,%.3f,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%.4f\n",
			(long)epoch,
			iso,
			elapsed_s,
			pack_v,
			cell_v,
			jack_v,
			battery_present ? 1 : 0,
			external_power ? 1 : 0,
			above_explore ? 1 : 0,
			status,
			m0_cmd,
			m1_cmd,
			direction,
			ax_mean,
			ay_mean,
			az_mean,
			amag_mean,
			amag_std,
			toggle_event,
			reversal_verified,
			reversal_score);
		fflush(fp);
		if(fdatasync(fd) < 0){
			perror("ERROR: fdatasync");
			rc_set_state(EXITING);
		}

		printf("%s m0=%+.2f m1=%+.2f pack=%.2fV cell=%.2fV jack=%.2fV amag_std=%.3f toggle=%d rev_ok=%d rev_score=%.3f status=%s\n",
			iso, m0_cmd, m1_cmd, pack_v, cell_v, jack_v, amag_std, toggle_event, reversal_verified, reversal_score, status);

		if(cfg.heartbeat_enabled){
			write_heartbeat(cfg.heartbeat_path, "RUNNING", status, elapsed_s, direction, m0_cmd, m1_cmd);
		}

		if(strcmp(status, "EXTERNAL_POWER") == 0){
			printf("Stopping: external power detected\n");
			if(cfg.heartbeat_enabled){
				write_heartbeat(cfg.heartbeat_path, "STOP", "EXTERNAL_POWER", elapsed_s, direction, m0_cmd, m1_cmd);
			}
			rc_set_state(EXITING);
		}
		if(strcmp(status, "STOP_LOW_BATTERY") == 0){
			printf("Stopping: low battery threshold reached\n");
			if(cfg.heartbeat_enabled){
				write_heartbeat(cfg.heartbeat_path, "STOP", "STOP_LOW_BATTERY", elapsed_s, direction, m0_cmd, m1_cmd);
			}
			rc_set_state(EXITING);
		}
		if(consecutive_unverified_toggles >= cfg.max_unverified_toggles){
			printf("Stopping: reversal not verified for %d toggles\n", consecutive_unverified_toggles);
			emergency_brake_and_stop(cfg.motor_ch,
				"reversal verification failed repeatedly",
				cfg.heartbeat_path,
				cfg.heartbeat_enabled,
				elapsed_s,
				direction,
				m0_cmd,
				m1_cmd);
		}

		if(sleep_until(&wake, cfg.interval_sec) < 0){
			rc_set_state(EXITING);
		}
	}

	pthread_mutex_lock(&motor_state.lock);
	motor_state.running = 0;
	pthread_mutex_unlock(&motor_state.lock);
	pthread_join(motor_thread, NULL);
	pthread_mutex_destroy(&motor_state.lock);

	rc_motor_brake(cfg.motor_ch);
	rc_motor_cleanup();
	rc_mpu_power_off();
	if(cfg.use_adc) rc_adc_cleanup();
	fclose(fp);
	return 0;
}
