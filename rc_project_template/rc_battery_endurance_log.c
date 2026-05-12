/**
 * @file rc_battery_endurance_log.c
 *
 * Logs battery and DC jack voltage at fixed intervals for endurance testing.
 * Data is written with fdatasync() each sample to reduce the chance of
 * losing buffered data if power disappears unexpectedly.
 */

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <rc/adc.h>
#include <rc/start_stop.h>
#include <rc/time.h>

#define DEFAULT_LOG_PATH "battery_endurance.csv"
#define DEFAULT_INTERVAL_SEC 10
#define DEFAULT_SAMPLES 10
#define DEFAULT_DISCONNECT_V 1.0
#define DEFAULT_MIN_CELL_V 3.95
#define DEFAULT_STOP_CELL_V 3.30
#define DEFAULT_MAX_JACK_V 1.0

struct config_t {
	const char* log_path;
	int interval_sec;
	int samples;
	double disconnect_v;
	double min_cell_v;
	double stop_cell_v;
	double max_jack_v;
	bool require_no_external;
};

static void print_usage(const char* prog)
{
	printf("Usage: %s [options]\n", prog);
	printf("Options:\n");
	printf("  --log PATH           Output CSV path (default %s)\n", DEFAULT_LOG_PATH);
	printf("  --interval-sec N     Fixed logging interval in seconds (default %d)\n", DEFAULT_INTERVAL_SEC);
	printf("  --samples N          ADC samples averaged per log row (default %d)\n", DEFAULT_SAMPLES);
	printf("  --cell-min V         Exploration threshold per cell (default %.2fV)\n", DEFAULT_MIN_CELL_V);
	printf("  --cell-stop V        Stop test when cell <= V (default %.2fV)\n", DEFAULT_STOP_CELL_V);
	printf("  --jack-max V         Max jack voltage considered unplugged (default %.2fV)\n", DEFAULT_MAX_JACK_V);
	printf("  --disconnect-v V     Below this pack/jack voltage, treat as disconnected (default %.2fV)\n", DEFAULT_DISCONNECT_V);
	printf("  --allow-external     Continue logging even if external power is detected\n");
	printf("  --help               Show this help\n");
}

static int parse_int_arg(const char* flag, const char* val, int* out)
{
	char* end = NULL;
	long n = strtol(val, &end, 10);
	if(end == val || *end != '\0' || n < 1 || n > 86400){
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
			if(i + 1 >= argc || parse_int_arg("--interval-sec", argv[++i], &cfg->interval_sec) < 0){
				return -1;
			}
		}
		else if(strcmp(argv[i], "--samples") == 0){
			if(i + 1 >= argc || parse_int_arg("--samples", argv[++i], &cfg->samples) < 0){
				return -1;
			}
		}
		else if(strcmp(argv[i], "--cell-min") == 0){
			if(i + 1 >= argc || parse_double_arg("--cell-min", argv[++i], &cfg->min_cell_v) < 0){
				return -1;
			}
		}
		else if(strcmp(argv[i], "--cell-stop") == 0){
			if(i + 1 >= argc || parse_double_arg("--cell-stop", argv[++i], &cfg->stop_cell_v) < 0){
				return -1;
			}
		}
		else if(strcmp(argv[i], "--jack-max") == 0){
			if(i + 1 >= argc || parse_double_arg("--jack-max", argv[++i], &cfg->max_jack_v) < 0){
				return -1;
			}
		}
		else if(strcmp(argv[i], "--disconnect-v") == 0){
			if(i + 1 >= argc || parse_double_arg("--disconnect-v", argv[++i], &cfg->disconnect_v) < 0){
				return -1;
			}
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

	if(cfg->samples < 1 || cfg->interval_sec < 1){
		fprintf(stderr, "ERROR: interval and samples must be >= 1\n");
		return -1;
	}
	if(cfg->min_cell_v <= 0 || cfg->stop_cell_v <= 0 || cfg->max_jack_v < 0 || cfg->disconnect_v < 0){
		fprintf(stderr, "ERROR: threshold values are invalid\n");
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
		perror("ERROR: stat log file");
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
		fprintf(fp, "epoch_s,iso8601,elapsed_s,pack_v,cell_v,dc_jack_v,battery_present,external_power,above_explore_threshold,status\n");
		fflush(fp);
		fdatasync(fd);
	}

	*out_fp = fp;
	*out_fd = fd;
	return 0;
}

static int sleep_until(const struct timespec* ts)
{
	while(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts, NULL) == -1){
		if(errno == EINTR){
			continue;
		}
		perror("ERROR: clock_nanosleep");
		return -1;
	}
	return 0;
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
		.require_no_external = true,
	};

	int parse = parse_args(argc, argv, &cfg);
	if(parse == 1){
		return 0;
	}
	if(parse < 0){
		print_usage(argv[0]);
		return 1;
	}

	if(rc_enable_signal_handler() == -1){
		fprintf(stderr, "ERROR: failed to start signal handler\n");
		return 1;
	}
	if(rc_adc_init() == -1){
		fprintf(stderr, "ERROR: failed to initialize ADC\n");
		return 1;
	}

	FILE* fp = NULL;
	int fd = -1;
	if(open_log_file(cfg.log_path, &fp, &fd) < 0){
		rc_adc_cleanup();
		return 1;
	}

	printf("Logging to: %s\n", cfg.log_path);
	printf("Interval: %ds, samples/check: %d\n", cfg.interval_sec, cfg.samples);
	printf("Thresholds: cell_min=%.2fV cell_stop=%.2fV jack_max=%.2fV disconnect=%.2fV\n",
		cfg.min_cell_v, cfg.stop_cell_v, cfg.max_jack_v, cfg.disconnect_v);
	if(cfg.require_no_external){
		printf("External power must be disconnected for endurance run.\n");
	}
	else{
		printf("External power allowed (logging only mode).\n");
	}

	struct timespec t0 = {0};
	struct timespec next = {0};
	clock_gettime(CLOCK_MONOTONIC, &t0);
	next = t0;
	rc_set_state(RUNNING);

	while(rc_get_state() != EXITING){
		double pack_sum = 0.0;
		double jack_sum = 0.0;

		for(int i = 0; i < cfg.samples; i++){
			double p = rc_adc_batt();
			double j = rc_adc_dc_jack();
			if(p < 0.0 || j < 0.0){
				fprintf(stderr, "ERROR: ADC read failed\n");
				rc_set_state(EXITING);
				break;
			}
			pack_sum += p;
			jack_sum += j;
			rc_usleep(10000);
		}
		if(rc_get_state() == EXITING){
			break;
		}

		double pack_v = pack_sum / cfg.samples;
		double jack_v = jack_sum / cfg.samples;
		bool battery_present = pack_v >= cfg.disconnect_v;
		double cell_v = battery_present ? pack_v / 2.0 : 0.0;
		bool external_power = jack_v >= cfg.max_jack_v;
		bool above_explore = battery_present && (cell_v >= cfg.min_cell_v);

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

		time_t epoch = time(NULL);
		struct tm tmbuf;
		char iso[32] = {0};
		gmtime_r(&epoch, &tmbuf);
		strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%SZ", &tmbuf);

		struct timespec now_mono;
		clock_gettime(CLOCK_MONOTONIC, &now_mono);
		double elapsed_s = (double)(now_mono.tv_sec - t0.tv_sec) +
			((double)(now_mono.tv_nsec - t0.tv_nsec) / 1e9);

		fprintf(fp, "%ld,%s,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%s\n",
			(long)epoch,
			iso,
			elapsed_s,
			pack_v,
			cell_v,
			jack_v,
			battery_present ? 1 : 0,
			external_power ? 1 : 0,
			above_explore ? 1 : 0,
			status);
		fflush(fp);
		if(fdatasync(fd) < 0){
			perror("ERROR: fdatasync");
			rc_set_state(EXITING);
		}

		printf("%s pack=%.2fV cell=%.2fV jack=%.2fV status=%s\n",
			iso, pack_v, cell_v, jack_v, status);

		if(strcmp(status, "STOP_LOW_BATTERY") == 0){
			printf("Stopping: cell voltage reached stop threshold %.2fV\n", cfg.stop_cell_v);
			rc_set_state(EXITING);
		}
		if(strcmp(status, "EXTERNAL_POWER") == 0){
			printf("Stopping: external power detected (jack %.2fV >= %.2fV)\n", jack_v, cfg.max_jack_v);
			rc_set_state(EXITING);
		}

		next.tv_sec += cfg.interval_sec;
		if(sleep_until(&next) < 0){
			rc_set_state(EXITING);
		}
	}

	fclose(fp);
	rc_adc_cleanup();
	return 0;
}
