/**
 * @file rc_explore_preflight.c
 *
 * ADC-based preflight check for exploration mode:
 * 1) Battery must be present and charged above threshold.
 * 2) External DC jack power must NOT be connected.
 *
 * Exit codes:
 *   0 = PASS (safe to explore)
 *   2 = FAIL preflight conditions
 *   1 = runtime/configuration error
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <rc/adc.h>
#include <rc/start_stop.h>
#include <rc/time.h>

#define DEFAULT_DISCONNECT_V 1.0
#define DEFAULT_MIN_CELL_V 3.95
#define DEFAULT_MAX_JACK_V 1.0
#define DEFAULT_SAMPLES 20
#define DEFAULT_WATCH_PERIOD_MS 500

static void print_usage(const char* prog)
{
	printf("Usage: %s [options]\n", prog);
	printf("Options:\n");
	printf("  --cell-min V      Minimum per-cell voltage (default %.2f)\n", DEFAULT_MIN_CELL_V);
	printf("  --jack-max V      Maximum allowed DC jack voltage for unplugged state (default %.2f)\n", DEFAULT_MAX_JACK_V);
	printf("  --samples N       Number of ADC samples to average (default %d)\n", DEFAULT_SAMPLES);
	printf("  --watch           Keep monitoring until Ctrl+C\n");
	printf("  --period-ms N     Watch update period in ms (default %d)\n", DEFAULT_WATCH_PERIOD_MS);
	printf("  --help            Show this message\n");
}

int main(int argc, char* argv[])
{
	double min_cell_v = DEFAULT_MIN_CELL_V;
	double max_jack_v = DEFAULT_MAX_JACK_V;
	double disconnect_v = DEFAULT_DISCONNECT_V;
	int samples = DEFAULT_SAMPLES;
	int period_ms = DEFAULT_WATCH_PERIOD_MS;
	bool watch_mode = false;

	for(int i = 1; i < argc; i++){
		if(strcmp(argv[i], "--cell-min") == 0){
			if(i + 1 >= argc){
				fprintf(stderr, "ERROR: --cell-min requires a value\n");
				return 1;
			}
			min_cell_v = atof(argv[++i]);
		}
		else if(strcmp(argv[i], "--jack-max") == 0){
			if(i + 1 >= argc){
				fprintf(stderr, "ERROR: --jack-max requires a value\n");
				return 1;
			}
			max_jack_v = atof(argv[++i]);
		}
		else if(strcmp(argv[i], "--samples") == 0){
			if(i + 1 >= argc){
				fprintf(stderr, "ERROR: --samples requires a value\n");
				return 1;
			}
			samples = atoi(argv[++i]);
		}
		else if(strcmp(argv[i], "--watch") == 0){
			watch_mode = true;
		}
		else if(strcmp(argv[i], "--period-ms") == 0){
			if(i + 1 >= argc){
				fprintf(stderr, "ERROR: --period-ms requires a value\n");
				return 1;
			}
			period_ms = atoi(argv[++i]);
		}
		else if(strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0){
			print_usage(argv[0]);
			return 0;
		}
		else{
			fprintf(stderr, "ERROR: unknown argument '%s'\n", argv[i]);
			print_usage(argv[0]);
			return 1;
		}
	}

	if(samples < 1 || period_ms < 1 || min_cell_v <= 0.0 || max_jack_v < 0.0){
		fprintf(stderr, "ERROR: invalid argument values\n");
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

	printf("Preflight thresholds: cell >= %.2fV, dc_jack < %.2fV\n", min_cell_v, max_jack_v);
	printf("Sampling: %d samples per check\n", samples);
	if(watch_mode){
		printf("Watch mode enabled. Press Ctrl+C to exit.\n");
	}

	rc_set_state(RUNNING);
	int exit_code = 2;

	while(rc_get_state() != EXITING){
		double pack_sum = 0.0;
		double jack_sum = 0.0;

		for(int i = 0; i < samples; i++){
			double pack = rc_adc_batt();
			double jack = rc_adc_dc_jack();
			if(pack < 0.0 || jack < 0.0){
				fprintf(stderr, "ERROR: failed to read ADC voltages\n");
				exit_code = 1;
				goto cleanup;
			}
			pack_sum += pack;
			jack_sum += jack;
			rc_usleep(10000);
		}

		double pack_v = pack_sum / samples;
		double jack_v = jack_sum / samples;
		bool battery_present = pack_v >= disconnect_v;
		double cell_v = battery_present ? (pack_v / 2.0) : 0.0;
		bool battery_charged = battery_present && (cell_v >= min_cell_v);
		bool jack_connected = jack_v >= max_jack_v;
		bool safe_to_explore = battery_charged && !jack_connected;

		printf("\rPack: %5.2fV  Cell: %4.2fV  DC Jack: %5.2fV  => %s", pack_v, cell_v, jack_v,
			safe_to_explore ? "PASS" : "FAIL");
		fflush(stdout);

		if(!watch_mode){
			printf("\n");
			if(!battery_present){
				printf("Reason: battery not detected on balance connector\n");
			}
			else if(!battery_charged){
				printf("Reason: battery below threshold (cell %.2fV < %.2fV)\n", cell_v, min_cell_v);
			}
			else if(jack_connected){
				printf("Reason: external power detected on DC jack (%.2fV)\n", jack_v);
			}

			exit_code = safe_to_explore ? 0 : 2;
			goto cleanup;
		}

		rc_usleep(period_ms * 1000);
	}

cleanup:
	printf("\n");
	rc_adc_cleanup();
	return exit_code;
}
