/**
 * @file rc_robot_server.c
 *
 * TCP command server for remote robot control.
 *
 * Heading correction architecture:
 *   - A 20 Hz control thread continuously reads the gyro heading and applies
 *     a proportional differential correction to keep the robot on a straight line.
 *   - On every FORWARD/BACKWARD command the magnetometer is sampled (motors are
 *     still at 0, so no motor EMI) to reseed the gyro heading to an absolute
 *     world-frame reference.  This prevents gyro drift from compounding across
 *     stop/start cycles on smooth floors.
 *   - The HOME command can be sent at any time while stationary to reset the
 *     heading reference to the current compass bearing.
 *
 * Commands (newline-terminated over TCP):
 *   FORWARD <duty>       Drive forward with heading correction
 *   BACKWARD <duty>      Drive backward with heading correction
 *   TURN <duty> <deg>    Gyro-guided in-place point turn
 *   STOP                 Soft ramp to zero
 *   BRAKE                Immediate emergency brake
 *   HOME                 Read compass; reseed gyro & set new heading reference
 *   STATUS               One-line JSON with heading, compass, motor state
 *   SET_ACCEL <rate>     Acceleration rate (duty/sec)
 *   SET_DECEL <rate>     Deceleration rate (duty/sec)
 *   SET_GAIN <kp>        Heading correction proportional gain
 *   SET_TOL <deg>        Heading error tolerance in degrees
 */

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>     // strcasecmp
#include <strings.h>    // strcasecmp on some platforms
#include <sys/socket.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>     // read, close

/** nanosleep wrapper replacing usleep (removed from POSIX.1-2008). */
static void us_sleep(int us) {
    struct timespec ts;
    ts.tv_sec  = us / 1000000;
    ts.tv_nsec = (us % 1000000) * 1000L;
    nanosleep(&ts, NULL);
}
#include <signal.h>

#include "rc_motor_ramp.h"
#include "rc_gyro_heading.h"

/* -------------------------------------------------------------------------
 * Defaults
 * ---------------------------------------------------------------------- */
#define DEFAULT_PORT              5000
#define DEFAULT_MAX_DUTY          0.6
#define DEFAULT_ACCEL_RATE        0.5
#define DEFAULT_DECEL_RATE        1.0
#define DEFAULT_HEADING_GAIN      0.15
#define DEFAULT_STRAIGHT_TOL_DEG  2.0
#define DEFAULT_WATCHDOG_SEC      5
#define CONTROL_THREAD_HZ         20
#define TURN_TOLERANCE_DEG        2.0
#define TURN_TIMEOUT_SEC          10
#define TURN_SLOWDOWN_DEG         90.0
#define TURN_MIN_DUTY             0.04
#define TURN_NEAR_TARGET_DEG      8.0
#define TURN_SETTLE_SAMPLES       3

/* -------------------------------------------------------------------------
 * Types
 * ---------------------------------------------------------------------- */
typedef struct {
    double ref_heading_deg;
    float  base_duty;
    double kp;
    double tolerance_deg;
    int    active;
} straight_t;

typedef struct {
    motor_ramp_t     motor_ramp;
    gyro_heading_t   heading;
    straight_t       straight;
    pthread_mutex_t  ctrl_mutex;
    pthread_t        ctrl_thread;
    time_t           last_cmd_time;
    int              watchdog_sec;
    int              port;
    int              server_fd;
    int              client_fd;
} srv_t;

/* -------------------------------------------------------------------------
 * Globals
 * ---------------------------------------------------------------------- */
volatile int g_shutdown = 0;

static void signal_handler(int sig) { (void)sig; g_shutdown = 1; }

/* -------------------------------------------------------------------------
 * Helpers
 * ---------------------------------------------------------------------- */
static double wrap_angle(double deg)
{
    while (deg >  180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
}

static double reseed_heading(srv_t *s)
{
    if (gyro_heading_compass_valid(&s->heading)) {
        gyro_heading_reseed_from_compass(&s->heading);
        double h = gyro_heading_get_deg(&s->heading);
        printf("[server] heading reseeded from compass: %.1f deg\n", h);
        return h;
    }
    double h = gyro_heading_get_deg(&s->heading);
    printf("[server] compass not yet valid; using gyro heading: %.1f deg\n", h);
    return h;
}

/* -------------------------------------------------------------------------
 * 20 Hz heading-correction control thread
 * ---------------------------------------------------------------------- */
static void* control_thread_func(void *arg)
{
    srv_t *s = (srv_t *)arg;
    const int interval_us = 1000000 / CONTROL_THREAD_HZ;

    while (!g_shutdown) {
        us_sleep(interval_us);

        pthread_mutex_lock(&s->ctrl_mutex);
        int    active   = s->straight.active;
        float  base     = s->straight.base_duty;
        double ref      = s->straight.ref_heading_deg;
        double kp       = s->straight.kp;
        double tol      = s->straight.tolerance_deg;
        float  max_duty = s->motor_ramp.max_duty;
        pthread_mutex_unlock(&s->ctrl_mutex);

        if (!active) continue;

        double heading = gyro_heading_get_deg(&s->heading);
        double error   = wrap_angle(heading - ref);

        float left, right;
        if (fabs(error) <= tol) {
            left  = base;
            right = base;
        } else {
            float corr = (float)(kp * error);
            float max_corr = 0.3f * max_duty;
            if (corr >  max_corr) corr =  max_corr;
            if (corr < -max_corr) corr = -max_corr;
            left  = base + corr;
            right = base - corr;
            if (left  >  max_duty) left  =  max_duty;
            if (left  < -max_duty) left  = -max_duty;
            if (right >  max_duty) right =  max_duty;
            if (right < -max_duty) right = -max_duty;
        }

        motor_ramp_set_target(&s->motor_ramp, left, right);
    }
    return NULL;
}

/* -------------------------------------------------------------------------
 * Command implementations
 * ---------------------------------------------------------------------- */
static void cmd_forward(srv_t *s, float duty)
{
    duty = fmaxf(0.0f, fminf(s->motor_ramp.max_duty, duty));
    double ref = reseed_heading(s);
    pthread_mutex_lock(&s->ctrl_mutex);
    s->straight.ref_heading_deg = ref;
    s->straight.base_duty       = duty;
    s->straight.active          = 1;
    pthread_mutex_unlock(&s->ctrl_mutex);
    // Kick the ramp immediately; control thread will adjust for drift on each tick
    motor_ramp_set_target(&s->motor_ramp, duty, duty);
}

static void cmd_backward(srv_t *s, float duty)
{
    duty = fmaxf(0.0f, fminf(s->motor_ramp.max_duty, duty));
    double ref = reseed_heading(s);
    pthread_mutex_lock(&s->ctrl_mutex);
    s->straight.ref_heading_deg = ref;
    s->straight.base_duty       = -duty;
    s->straight.active          = 1;
    pthread_mutex_unlock(&s->ctrl_mutex);
    // Kick the ramp immediately; control thread will adjust for drift on each tick
    motor_ramp_set_target(&s->motor_ramp, -duty, -duty);
}

static void cmd_stop(srv_t *s)
{
    pthread_mutex_lock(&s->ctrl_mutex);
    s->straight.active    = 0;
    s->straight.base_duty = 0.0f;
    pthread_mutex_unlock(&s->ctrl_mutex);
    motor_ramp_stop(&s->motor_ramp);
}

static void cmd_brake(srv_t *s)
{
    pthread_mutex_lock(&s->ctrl_mutex);
    s->straight.active    = 0;
    s->straight.base_duty = 0.0f;
    pthread_mutex_unlock(&s->ctrl_mutex);
    motor_ramp_brake(&s->motor_ramp);
}

static void cmd_turn(srv_t *s, float duty, float deg)
{
    duty = fmaxf(0.0f, fminf(s->motor_ramp.max_duty, duty));
    pthread_mutex_lock(&s->ctrl_mutex);
    s->straight.active    = 0;
    s->straight.base_duty = 0.0f;
    pthread_mutex_unlock(&s->ctrl_mutex);

    double last_heading = gyro_heading_get_deg(&s->heading);
    double turned_deg = 0.0;

    if (duty <= 0.0f || fabs(deg) <= TURN_TOLERANCE_DEG) {
        motor_ramp_brake(&s->motor_ramp);
        reseed_heading(s);
        return;
    }

    int settle_count = 0;
    time_t t0 = time(NULL);
    while (!g_shutdown && (time(NULL) - t0) < TURN_TIMEOUT_SEC) {
        double heading = gyro_heading_get_deg(&s->heading);
        double step = wrap_angle(heading - last_heading);
        last_heading = heading;
        turned_deg += step;

        double remaining = (double)deg - turned_deg;
        double abs_err = fabs(remaining);

        if (abs_err <= TURN_TOLERANCE_DEG) {
            settle_count++;
            motor_ramp_set_target(&s->motor_ramp, 0.0f, 0.0f);
            if (settle_count >= TURN_SETTLE_SAMPLES) break;
            us_sleep(25000);
            continue;
        }
        settle_count = 0;

        double scale = abs_err / TURN_SLOWDOWN_DEG;
        if (scale > 1.0) scale = 1.0;
        float cmd = (float)(duty * scale);
        if (abs_err > TURN_NEAR_TARGET_DEG && cmd < (float)TURN_MIN_DUTY) {
            cmd = (float)TURN_MIN_DUTY;
        }
        if (cmd > duty) cmd = duty;

        float left_duty  = (remaining > 0.0) ? -cmd :  cmd;
        float right_duty = (remaining > 0.0) ?  cmd : -cmd;
        motor_ramp_set_target(&s->motor_ramp, left_duty, right_duty);
        us_sleep(25000);
    }
    motor_ramp_brake(&s->motor_ramp);
    reseed_heading(s);
}

static void cmd_home(srv_t *s)
{
    double ref = reseed_heading(s);
    pthread_mutex_lock(&s->ctrl_mutex);
    s->straight.ref_heading_deg = ref;
    pthread_mutex_unlock(&s->ctrl_mutex);
    printf("[server] HOME: reference set to %.1f deg\n", ref);
}

/* -------------------------------------------------------------------------
 * Command dispatcher
 * ---------------------------------------------------------------------- */
static void process_command(srv_t *s, const char *line,
                            char *response, int resp_len)
{
    char cmd[64];
    if (sscanf(line, "%63s", cmd) != 1) {
        snprintf(response, resp_len, "ERR empty command\n"); return;
    }
    s->last_cmd_time = time(NULL);

    if (strcasecmp(cmd, "FORWARD") == 0) {
        float duty = 0.0f;
        if (sscanf(line, "FORWARD %f", &duty) != 1) {
            snprintf(response, resp_len, "ERR FORWARD <duty>\n"); return; }
        cmd_forward(s, duty);
        snprintf(response, resp_len, "OK\n");

    } else if (strcasecmp(cmd, "BACKWARD") == 0) {
        float duty = 0.0f;
        if (sscanf(line, "BACKWARD %f", &duty) != 1) {
            snprintf(response, resp_len, "ERR BACKWARD <duty>\n"); return; }
        cmd_backward(s, duty);
        snprintf(response, resp_len, "OK\n");

    } else if (strcasecmp(cmd, "TURN") == 0) {
        float duty = 0.0f, deg = 0.0f;
        if (sscanf(line, "TURN %f %f", &duty, &deg) != 2) {
            snprintf(response, resp_len, "ERR TURN <duty> <deg>\n"); return; }
        cmd_turn(s, duty, deg);
        snprintf(response, resp_len, "OK\n");

    } else if (strcasecmp(cmd, "STOP") == 0) {
        cmd_stop(s);
        snprintf(response, resp_len, "OK\n");

    } else if (strcasecmp(cmd, "BRAKE") == 0) {
        cmd_brake(s);
        snprintf(response, resp_len, "OK\n");

    } else if (strcasecmp(cmd, "HOME") == 0) {
        cmd_home(s);
        snprintf(response, resp_len, "OK\n");

    } else if (strcasecmp(cmd, "STATUS") == 0) {
        float left = 0.0f, right = 0.0f;
        float last_m0_cmd = 0.0f, last_m1_cmd = 0.0f;
        int last_m0_ret = 0, last_m1_ret = 0;
        double mag_x = 0.0, mag_y = 0.0, mag_z = 0.0;
        int mag_samples = 0, mag_errors = 0;
        motor_ramp_get_current(&s->motor_ramp, &left, &right);
        motor_ramp_get_driver_status(&s->motor_ramp, &last_m0_cmd, &last_m1_cmd,
                                     &last_m0_ret, &last_m1_ret);
        double heading     = gyro_heading_get_deg(&s->heading);
        double gyro_rate   = gyro_heading_get_rate_dps(&s->heading);
        double compass_deg = gyro_heading_get_compass_deg(&s->heading);
        int    comp_valid  = gyro_heading_compass_valid(&s->heading);
        gyro_heading_get_mag_raw(&s->heading, &mag_x, &mag_y, &mag_z);
        gyro_heading_get_mag_counters(&s->heading, &mag_samples, &mag_errors);
        pthread_mutex_lock(&s->ctrl_mutex);
        double ref_heading = s->straight.ref_heading_deg;
        double base_duty   = (double)s->straight.base_duty;
        int    active      = s->straight.active;
        pthread_mutex_unlock(&s->ctrl_mutex);
        snprintf(response, resp_len,
            "{\"heading_deg\":%.2f,\"gyro_rate_dps\":%.2f,"
            "\"compass_deg\":%.2f,\"compass_valid\":%d,"
            "\"mag_x_ut\":%.2f,\"mag_y_ut\":%.2f,\"mag_z_ut\":%.2f,"
            "\"mag_samples\":%d,\"mag_errors\":%d,"
            "\"ref_heading_deg\":%.2f,\"base_duty\":%.2f,"
            "\"correction_active\":%d,"
            "\"motor_left\":%.2f,\"motor_right\":%.2f,"
            "\"motor_m0_cmd\":%.2f,\"motor_m1_cmd\":%.2f,"
            "\"motor_m0_ret\":%d,\"motor_m1_ret\":%d}\n",
            heading, gyro_rate, compass_deg, comp_valid,
            mag_x, mag_y, mag_z, mag_samples, mag_errors,
            ref_heading, base_duty, active,
            (double)left, (double)right,
            (double)last_m0_cmd, (double)last_m1_cmd,
            last_m0_ret, last_m1_ret);

    } else if (strcasecmp(cmd, "SET_ACCEL") == 0) {
        float rate = 0.0f;
        if (sscanf(line, "SET_ACCEL %f", &rate) != 1) {
            snprintf(response, resp_len, "ERR SET_ACCEL <rate>\n"); return; }
        motor_ramp_set_rates(&s->motor_ramp, rate, s->motor_ramp.decel_rate);
        snprintf(response, resp_len, "OK\n");

    } else if (strcasecmp(cmd, "SET_DECEL") == 0) {
        float rate = 0.0f;
        if (sscanf(line, "SET_DECEL %f", &rate) != 1) {
            snprintf(response, resp_len, "ERR SET_DECEL <rate>\n"); return; }
        motor_ramp_set_rates(&s->motor_ramp, s->motor_ramp.accel_rate, rate);
        snprintf(response, resp_len, "OK\n");

    } else if (strcasecmp(cmd, "SET_GAIN") == 0) {
        float gain = 0.0f;
        if (sscanf(line, "SET_GAIN %f", &gain) != 1) {
            snprintf(response, resp_len, "ERR SET_GAIN <kp>\n"); return; }
        pthread_mutex_lock(&s->ctrl_mutex);
        s->straight.kp = (double)gain;
        pthread_mutex_unlock(&s->ctrl_mutex);
        snprintf(response, resp_len, "OK\n");

    } else if (strcasecmp(cmd, "SET_TOL") == 0) {
        float tol = 0.0f;
        if (sscanf(line, "SET_TOL %f", &tol) != 1) {
            snprintf(response, resp_len, "ERR SET_TOL <deg>\n"); return; }
        pthread_mutex_lock(&s->ctrl_mutex);
        s->straight.tolerance_deg = (double)tol;
        pthread_mutex_unlock(&s->ctrl_mutex);
        snprintf(response, resp_len, "OK\n");

    } else {
        snprintf(response, resp_len, "ERR unknown: %s\n", cmd);
    }
}

/* -------------------------------------------------------------------------
 * Usage
 * ---------------------------------------------------------------------- */
static void print_usage(const char *prog)
{
    printf("Usage: %s [options]\n", prog);
    printf("  --port N           TCP port (default %d)\n", DEFAULT_PORT);
    printf("  --max-duty D       Max duty cycle (default %.2f)\n", DEFAULT_MAX_DUTY);
    printf("  --accel-rate R     Accel rate duty/sec (default %.2f)\n", DEFAULT_ACCEL_RATE);
    printf("  --decel-rate R     Decel rate duty/sec (default %.2f)\n", DEFAULT_DECEL_RATE);
    printf("  --heading-gain K   Straight-line correction gain (default %.2f)\n", DEFAULT_HEADING_GAIN);
    printf("  --heading-tol T    Dead-band degrees (default %.1f)\n", DEFAULT_STRAIGHT_TOL_DEG);
    printf("  --watchdog-sec N   Client silence timeout (default %d)\n", DEFAULT_WATCHDOG_SEC);
    printf("  --help\n");
}

/* -------------------------------------------------------------------------
 * main
 * ---------------------------------------------------------------------- */
int main(int argc, char *argv[])
{
    srv_t s;
    memset(&s, 0, sizeof(s));
    s.port                   = DEFAULT_PORT;
    s.motor_ramp.max_duty    = (float)DEFAULT_MAX_DUTY;
    s.motor_ramp.accel_rate  = (float)DEFAULT_ACCEL_RATE;
    s.motor_ramp.decel_rate  = (float)DEFAULT_DECEL_RATE;
    s.straight.kp            = DEFAULT_HEADING_GAIN;
    s.straight.tolerance_deg = DEFAULT_STRAIGHT_TOL_DEG;
    s.watchdog_sec           = DEFAULT_WATCHDOG_SEC;
    s.server_fd              = -1;
    s.client_fd              = -1;

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--help"))        { print_usage(argv[0]); return 0; }
        else if (!strcmp(argv[i], "--port")         && i+1<argc) s.port                   = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--max-duty")     && i+1<argc) s.motor_ramp.max_duty    = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--accel-rate")   && i+1<argc) s.motor_ramp.accel_rate  = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--decel-rate")   && i+1<argc) s.motor_ramp.decel_rate  = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--heading-gain") && i+1<argc) s.straight.kp            = atof(argv[++i]);
        else if (!strcmp(argv[i], "--heading-tol")  && i+1<argc) s.straight.tolerance_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--watchdog-sec") && i+1<argc) s.watchdog_sec           = atoi(argv[++i]);
        else { fprintf(stderr, "Unknown option: %s\n", argv[i]); print_usage(argv[0]); return 1; }
    }

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    if (motor_ramp_init(&s.motor_ramp, s.motor_ramp.max_duty,
                        s.motor_ramp.accel_rate, s.motor_ramp.decel_rate) != 0) {
        fprintf(stderr, "motor_ramp_init failed\n"); return 1;
    }
    if (gyro_heading_init(&s.heading) != 0) {
        fprintf(stderr, "gyro_heading_init failed\n");
        motor_ramp_cleanup(&s.motor_ramp); return 1;
    }

    printf("[server] waiting 500 ms for magnetometer to settle...\n");
    us_sleep(500000);

    if (pthread_mutex_init(&s.ctrl_mutex, NULL) != 0) {
        fprintf(stderr, "ctrl_mutex init failed\n"); goto cleanup_imu;
    }
    if (pthread_create(&s.ctrl_thread, NULL, control_thread_func, &s) != 0) {
        fprintf(stderr, "ctrl_thread create failed\n"); goto cleanup_mutex;
    }

    s.server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (s.server_fd < 0) { perror("socket"); goto cleanup_thread; }

    int opt = 1;
    setsockopt(s.server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons((uint16_t)s.port);
    if (bind(s.server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind"); goto cleanup_thread;
    }
    if (listen(s.server_fd, 1) < 0) {
        perror("listen"); goto cleanup_thread;
    }

    printf("[rc_robot_server] port=%d max_duty=%.2f accel=%.2f/s decel=%.2f/s "
           "gain=%.2f tol=%.1fdeg watchdog=%ds\n",
           s.port,
           (double)s.motor_ramp.max_duty, (double)s.motor_ramp.accel_rate,
           (double)s.motor_ramp.decel_rate,
           s.straight.kp, s.straight.tolerance_deg, s.watchdog_sec);

    while (!g_shutdown) {
        struct sockaddr_in cli;
        socklen_t cli_len = sizeof(cli);
        s.client_fd = accept(s.server_fd, (struct sockaddr *)&cli, &cli_len);
        if (s.client_fd < 0) { if (errno == EINTR) continue; perror("accept"); break; }
        printf("[server] client connected\n");
        s.last_cmd_time = time(NULL);
        cmd_brake(&s);

        char buf[256];
        int  buf_idx = 0;
        while (!g_shutdown) {
            if ((time(NULL) - s.last_cmd_time) > s.watchdog_sec) {
                printf("[server] watchdog timeout - braking\n");
                cmd_brake(&s);
                s.last_cmd_time = time(NULL);
            }
            fd_set rfds;
            struct timeval tv = {1, 0};
            FD_ZERO(&rfds);
            FD_SET(s.client_fd, &rfds);
            int sel = select(s.client_fd + 1, &rfds, NULL, NULL, &tv);
            if (sel < 0) { if (errno == EINTR) continue; break; }
            if (sel == 0) continue;
            int n = (int)read(s.client_fd, &buf[buf_idx],
                              (size_t)(sizeof(buf) - buf_idx - 1));
            if (n <= 0) { printf("[server] client disconnected\n"); break; }
            buf_idx += n;
            for (int i = 0; i < buf_idx; i++) {
                if (buf[i] != '\n') continue;
                buf[i] = '\0';
                char resp[512];
                process_command(&s, buf, resp, sizeof(resp));
                if (send(s.client_fd, resp, strlen(resp), 0) < 0) { n = 0; break; }
                memmove(buf, &buf[i+1], (size_t)(buf_idx - i - 1));
                buf_idx -= (i + 1);
                i = -1;
            }
            if (n <= 0) break;
        }
        close(s.client_fd);
        s.client_fd = -1;
        cmd_brake(&s);
    }

cleanup_thread:
    printf("[server] shutting down\n");
    g_shutdown = 1;
    pthread_join(s.ctrl_thread, NULL);
cleanup_mutex:
    pthread_mutex_destroy(&s.ctrl_mutex);
cleanup_imu:
    if (s.server_fd >= 0) close(s.server_fd);
    if (s.client_fd >= 0) close(s.client_fd);
    motor_ramp_cleanup(&s.motor_ramp);
    gyro_heading_cleanup(&s.heading);
    return 0;
}
