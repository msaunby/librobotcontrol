#define _USE_MATH_DEFINES
#include <math.h>
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
 *   POSN                 One-line JSON with x/y position, heading, compass
 *   SET_ACCEL <rate>     Acceleration rate (duty/sec)
 *   SET_DECEL <rate>     Deceleration rate (duty/sec)
 *   SET_GAIN <kp>        Heading correction proportional gain
 *   SET_TOL <deg>        Heading error tolerance in degrees
 *   COLLISION on         Enable collision detection (Forward/Backward/ROT)
 *   COLLISION off        Disable collision detection
 *   COLLISION threshold <t>  Set accel spike threshold above 1g in m/s^2
 *   COLLISION rotstall <min_rate_dps> <per_duty_dps> <hold_ms>
 *                        Set adaptive ROT stall detector parameters
 *                        (legacy form: COLLISION rotstall <rate_dps> <hold_ms>)
 *   COLLISION status     Report enabled state and threshold
 *
 * Collision detection:
 *   When enabled, Forward, Backward, and ROT moves are monitored at 50 Hz via the
 *   accelerometer already used by the heading thread.  If the total
 *   acceleration magnitude exceeds (9.81 + threshold) m/s^2 — indicating a
 *   sudden deceleration spike — motors brake immediately and the command
 *   returns "COLLISION" instead of "OK".
 *   Detection is inhibited for the first 400 ms of each move to avoid false
 *   positives from the motor ramp-up transient.
 */

#define _POSIX_C_SOURCE 200809L
#define _USE_MATH_DEFINES

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

// For ROT command
#define ROT_TOLERANCE_DEG         2.5
#define ROT_TIMEOUT_SEC           12
#define ROT_SETTLE_SAMPLES        3
#define ROT_SLOWDOWN_DEG          90.0
#define ROT_MIN_DUTY              0.08
#define ROT_DEBUG                 1
#define ROT_STALL_MIN_RATE_DPS    5.0   ///< Minimum ROT stall threshold (dps)
#define ROT_STALL_PER_DUTY_DPS   25.0   ///< Additional threshold per duty unit
#define ROT_STALL_HOLD_US      300000   ///< Low-rate duration required to trigger stall

// Collision detection defaults
#define DEFAULT_COLLISION_THRESHOLD_MS2  4.0f  ///< Spike above g threshold (m/s^2)
#define COLLISION_STARTUP_SKIP_US       400000  ///< Ignore first 400ms of move
#define COLLISION_POLL_INTERVAL_US       20000  ///< Poll at 50 Hz

// Calibration from measured run: 0.155 m at duty 0.10 over 6.0 s.
// --- Sensor-feedback closed-loop rotate ---
// Direction-specific travel targets from tape-mark tests.
// The robot has been stopping short of the mark, so start with a longer forward run.
#define FORWARD_DEFAULT_DISTANCE_M      0.155f
#define BACKWARD_DEFAULT_DISTANCE_M     0.155f

// Backward motion tends to over-travel at higher duty.
// Scale backward runtime from 1.0 at duty=0.10 down to 0.750 at duty=0.50.
#define BACKWARD_TIME_SCALE_AT_DUTY_010 1.0f
#define BACKWARD_TIME_SCALE_AT_DUTY_050 0.762f

/* -------------------------------------------------------------------------
 * Types
 * ---------------------------------------------------------------------- */
typedef struct {
    double x_m;
    double y_m;
    double heading_deg;
} pose_t;

typedef struct {
    double ref_heading_deg;
    float  base_duty;
    double kp;
    double tolerance_deg;
    int    active;
} straight_t;

typedef struct {
    double min_x_m;
    double max_x_m;
    double min_y_m;
    double max_y_m;
    int    enabled;
} geofence_t;

typedef struct {
    motor_ramp_t     motor_ramp;
    gyro_heading_t   heading;
    straight_t       straight;
    pose_t           pose;
    geofence_t       geofence;
    int              collision_detect;        ///< 1 if collision detection enabled
    float            collision_threshold_ms2; ///< Accel spike threshold above g
    float            rot_stall_min_rate_dps;  ///< ROT stall min rate threshold
    float            rot_stall_per_duty_dps;  ///< ROT stall rate gain per duty
    int              rot_stall_hold_us;       ///< ROT low-rate hold duration
    pthread_mutex_t  ctrl_mutex;
    pthread_t        ctrl_thread;
    time_t           last_cmd_time;
    int              watchdog_sec;
    int              port;
    int              server_fd;
    int              client_fd;
} srv_t;

// speed_at_full_duty = 0.155 / (0.10 * 6.0) = 0.2583 m/s
#define FORWARD_M_PER_SEC_AT_FULL_DUTY   0.2583f

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

static int is_point_in_geofence(geofence_t *g, double x, double y)
{
    if (!g->enabled) return 1;  // Always in if disabled
    return (x >= g->min_x_m && x <= g->max_x_m &&
            y >= g->min_y_m && y <= g->max_y_m);
}

static int would_move_exit_geofence(geofence_t *g, pose_t *p,
                                     double next_x, double next_y)
{
    if (!g->enabled) return 0;  // Never exits if disabled
    return !is_point_in_geofence(g, next_x, next_y);
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
static int run_fixed_distance_motion(srv_t *s, float duty, float direction)
{
    duty = fmaxf(0.05f, fminf(s->motor_ramp.max_duty, duty));
    double ref = reseed_heading(s);
    float distance_m = (direction > 0.0f) ? FORWARD_DEFAULT_DISTANCE_M
                                          : BACKWARD_DEFAULT_DISTANCE_M;
    // Compute time to travel distance at this duty, using calibrated speed.
    float speed_mps = FORWARD_M_PER_SEC_AT_FULL_DUTY * duty;
    if (speed_mps <= 0.0f) speed_mps = 0.01f;
    float run_time_s = distance_m / speed_mps;

    if (direction < 0.0f) {
        float min_duty = 0.10f;
        float max_duty = 0.50f;
        float t = (duty - min_duty) / (max_duty - min_duty);
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        float scale = BACKWARD_TIME_SCALE_AT_DUTY_010 +
                      t * (BACKWARD_TIME_SCALE_AT_DUTY_050 - BACKWARD_TIME_SCALE_AT_DUTY_010);
        run_time_s *= scale;
    }

    pthread_mutex_lock(&s->ctrl_mutex);
    s->straight.ref_heading_deg = ref;
    s->straight.base_duty       = direction * duty;
    s->straight.active          = 1;
    // Keep pose heading aligned with this move's reseeded heading.
    s->pose.heading_deg         = ref;
    pthread_mutex_unlock(&s->ctrl_mutex);
    // Start moving
    motor_ramp_set_target(&s->motor_ramp, direction * duty, direction * duty);
    // Poll at 50 Hz for the planned move duration, checking for collision.
    // Detection is skipped for the first COLLISION_STARTUP_SKIP_US to avoid
    // false positives from the motor ramp-up transient.
    int run_time_us = (int)(run_time_s * 1e6);
    int elapsed_us  = 0;
    int collision   = 0;
    while (elapsed_us < run_time_us && !g_shutdown) {
        int step = (elapsed_us + COLLISION_POLL_INTERVAL_US <= run_time_us)
                   ? COLLISION_POLL_INTERVAL_US
                   : (run_time_us - elapsed_us);
        us_sleep(step);
        elapsed_us += step;

        pthread_mutex_lock(&s->ctrl_mutex);
        int   detect = s->collision_detect;
        float thresh = s->collision_threshold_ms2;
        pthread_mutex_unlock(&s->ctrl_mutex);

        if (detect && elapsed_us >= COLLISION_STARTUP_SKIP_US) {
            double ax, ay, az;
            if (gyro_heading_get_accel(&s->heading, &ax, &ay, &az) == 0) {
                double total = sqrt(ax*ax + ay*ay + az*az);
                if (total > 9.81 + (double)thresh) {
                    printf("[collision] detected: |accel|=%.2f m/s^2 (threshold=9.81+%.2f)\n",
                           total, (double)thresh);
                    collision = 1;
                    break;
                }
            }
        }
    }
    // Stop after move (or collision). Brake immediately to avoid extra travel.
    float actual_time_s = (float)elapsed_us / 1.0e6f;
    pthread_mutex_lock(&s->ctrl_mutex);
    s->straight.active    = 0;
    s->straight.base_duty = 0.0f;

    // Update pose based on actual elapsed time (shorter if collision aborted early)
    double heading_rad = ref * M_PI / 180.0;
    float  pose_time   = collision ? actual_time_s : run_time_s;
    s->pose.x_m += (double)(pose_time * FORWARD_M_PER_SEC_AT_FULL_DUTY * duty) * cos(heading_rad) * (double)direction;
    s->pose.y_m += (double)(pose_time * FORWARD_M_PER_SEC_AT_FULL_DUTY * duty) * sin(heading_rad) * (double)direction;
    printf("[pose] after move: x=%.4f y=%.4f h=%.2f%s\n",
           s->pose.x_m, s->pose.y_m, s->pose.heading_deg,
           collision ? " (collision)" : "");

    pthread_mutex_unlock(&s->ctrl_mutex);
    motor_ramp_brake(&s->motor_ramp);
    us_sleep(50000);
    return collision;
}

// Move forward a fixed distance (in meters) at the given duty (speed)
static int cmd_forward(srv_t *s, float duty)
{
    return run_fixed_distance_motion(s, duty, 1.0f);
}

static int cmd_backward(srv_t *s, float duty)
{
    return run_fixed_distance_motion(s, duty, -1.0f);
}

static void cmd_pose(srv_t *s, char *response, size_t resp_len)
{
    pthread_mutex_lock(&s->ctrl_mutex);
    double x = s->pose.x_m;
    double y = s->pose.y_m;
    double h = s->pose.heading_deg;
    pthread_mutex_unlock(&s->ctrl_mutex);
    snprintf(response, resp_len,
        "{\"x_m\":%.4f,\"y_m\":%.4f,\"heading_deg\":%.2f}\n",
        x, y, h);
}

static void cmd_posn(srv_t *s, char *response, size_t resp_len)
{
    pthread_mutex_lock(&s->ctrl_mutex);
    double x = s->pose.x_m;
    double y = s->pose.y_m;
    double heading = s->pose.heading_deg;
    pthread_mutex_unlock(&s->ctrl_mutex);

    double compass = gyro_heading_get_compass_deg(&s->heading);

    snprintf(response, resp_len,
        "{\"x_m\":%.4f,\"y_m\":%.4f,\"heading_deg\":%.2f,\"compass_deg\":%.2f}\n",
        x, y, heading, compass);
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
    
    // Update heading in pose after turn completes
    double final_heading = gyro_heading_get_deg(&s->heading);
    pthread_mutex_lock(&s->ctrl_mutex);
    s->pose.heading_deg = final_heading;
    printf("[pose] after turn: h=%.2f\n", s->pose.heading_deg);
    pthread_mutex_unlock(&s->ctrl_mutex);
    
    reseed_heading(s);
}

static void cmd_home(srv_t *s)
{
    double ref = reseed_heading(s);
    pthread_mutex_lock(&s->ctrl_mutex);
    s->straight.ref_heading_deg = ref;
    s->pose.x_m = 0.0;
    s->pose.y_m = 0.0;
    s->pose.heading_deg = ref;
    pthread_mutex_unlock(&s->ctrl_mutex);
    printf("[server] HOME: reference set to %.1f deg, pose reset to origin\n", ref);
}

/* -------------------------------------------------------------------------
 * Command dispatcher
 * ---------------------------------------------------------------------- */
static int cmd_rot(srv_t *s, float duty, float deg)
{
    duty = fmaxf(0.0f, fminf(s->motor_ramp.max_duty, duty));
    pthread_mutex_lock(&s->ctrl_mutex);
    s->straight.active    = 0;
    s->straight.base_duty = 0.0f;
    pthread_mutex_unlock(&s->ctrl_mutex);

    double start_heading = gyro_heading_get_deg(&s->heading);
    double target = start_heading + deg;
    // Wrap target to [-180, 180]
    if (target > 180.0) target -= 360.0;
    if (target < -180.0) target += 360.0;

    if (duty <= 0.0f || fabs(deg) <= ROT_TOLERANCE_DEG) {
        motor_ramp_brake(&s->motor_ramp);
        reseed_heading(s);
        return 0;
    }

    // Scale duty by angle: small angles get less duty to avoid overshoot
    float scaled_duty = duty * (fabs(deg) / ROT_SLOWDOWN_DEG);
    scaled_duty = fmaxf(ROT_MIN_DUTY, fminf(duty, scaled_duty));

    int settle_count = 0;
    int iter = 0;
    int elapsed_us = 0;
    int stall_us = 0;
    int collision = 0;
    time_t t0 = time(NULL);
    while (!g_shutdown && (time(NULL) - t0) < ROT_TIMEOUT_SEC) {
        double heading = gyro_heading_get_deg(&s->heading);
        // Compute shortest path error
        double err = target - heading;
        if (err > 180.0) err -= 360.0;
        if (err < -180.0) err += 360.0;
        double abs_err = fabs(err);

        if (ROT_DEBUG && iter % 8 == 0) {
            printf("[ROT] iter=%d h=%.2f target=%.2f err=%.2f abs_err=%.2f settle=%d scaled_duty=%.3f\n",
                   iter, heading, target, err, abs_err, settle_count, scaled_duty);
        }
        iter++;

        pthread_mutex_lock(&s->ctrl_mutex);
        int   detect = s->collision_detect;
        float thresh = s->collision_threshold_ms2;
        float stall_min_rate_dps = s->rot_stall_min_rate_dps;
        float stall_per_duty_dps = s->rot_stall_per_duty_dps;
        int   stall_hold_us = s->rot_stall_hold_us;
        pthread_mutex_unlock(&s->ctrl_mutex);

        if (detect && elapsed_us >= COLLISION_STARTUP_SKIP_US) {
            double ax, ay, az;
            if (gyro_heading_get_accel(&s->heading, &ax, &ay, &az) == 0) {
                double total = sqrt(ax*ax + ay*ay + az*az);
                if (total > 9.81 + (double)thresh) {
                    printf("[collision] detected during ROT: |accel|=%.2f m/s^2 (threshold=9.81+%.2f)\n",
                           total, (double)thresh);
                    collision = 1;
                    break;
                }
            }

            // ROT is slower than linear moves. Also detect a likely collision
            // when commanded rotation is not producing expected angular rate.
            if (abs_err > (ROT_TOLERANCE_DEG * 2.0)) {
                double rate_dps = fabs(gyro_heading_get_rate_dps(&s->heading));
                double expected_min_rate_dps = fmax((double)stall_min_rate_dps,
                                                    (double)stall_per_duty_dps * (double)scaled_duty);
                if (rate_dps < expected_min_rate_dps) {
                    stall_us += 25000;
                    if (stall_us >= stall_hold_us) {
                        printf("[collision] detected during ROT stall: |gyro_z|=%.2f dps < %.2f dps for %d ms\n",
                               rate_dps, expected_min_rate_dps, stall_us / 1000);
                        collision = 1;
                        break;
                    }
                } else {
                    stall_us = 0;
                }
            } else {
                stall_us = 0;
            }
        }

        if (abs_err <= ROT_TOLERANCE_DEG) {
            settle_count++;
            motor_ramp_set_target(&s->motor_ramp, 0.0f, 0.0f);
            if (settle_count >= ROT_SETTLE_SAMPLES) break;
            us_sleep(25000);
            elapsed_us += 25000;
            continue;
        }
        settle_count = 0;

        float left_duty  = (err > 0.0) ? -scaled_duty :  scaled_duty;
        float right_duty = (err > 0.0) ?  scaled_duty : -scaled_duty;
        motor_ramp_set_target(&s->motor_ramp, left_duty, right_duty);
        us_sleep(25000);
        elapsed_us += 25000;
    }
    motor_ramp_brake(&s->motor_ramp);
    // Update heading in pose after turn completes
    double final_heading = gyro_heading_get_deg(&s->heading);
    pthread_mutex_lock(&s->ctrl_mutex);
    s->pose.heading_deg = final_heading;
    printf("[pose] after ROT: h=%.2f%s\n", s->pose.heading_deg,
           collision ? " (collision)" : "");
    pthread_mutex_unlock(&s->ctrl_mutex);
    reseed_heading(s);
    return collision;
}

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
        
        // Predict next position after forward move
        pthread_mutex_lock(&s->ctrl_mutex);
        double cur_x = s->pose.x_m;
        double cur_y = s->pose.y_m;
        double h_rad = s->pose.heading_deg * M_PI / 180.0;
        double next_x = cur_x + (FORWARD_DEFAULT_DISTANCE_M * cos(h_rad));
        double next_y = cur_y + (FORWARD_DEFAULT_DISTANCE_M * sin(h_rad));
        int blocked = would_move_exit_geofence(&s->geofence, &s->pose, next_x, next_y);
        pthread_mutex_unlock(&s->ctrl_mutex);
        
        if (blocked) {
            snprintf(response, resp_len, "BLOCK\n");
            printf("[geofence] FORWARD blocked at (%.4f, %.4f) -> (%.4f, %.4f)\n",
                   cur_x, cur_y, next_x, next_y);
        } else {
            int r = cmd_forward(s, duty);
            snprintf(response, resp_len, r ? "COLLISION\n" : "OK\n");
        }

    } else if (strcasecmp(cmd, "BACKWARD") == 0) {
        float duty = 0.0f;
        if (sscanf(line, "BACKWARD %f", &duty) != 1) {
            snprintf(response, resp_len, "ERR BACKWARD <duty>\n"); return; }
        
        // Predict next position after backward move
        pthread_mutex_lock(&s->ctrl_mutex);
        double cur_x = s->pose.x_m;
        double cur_y = s->pose.y_m;
        double h_rad = s->pose.heading_deg * M_PI / 180.0;
        double next_x = cur_x - (BACKWARD_DEFAULT_DISTANCE_M * cos(h_rad));
        double next_y = cur_y - (BACKWARD_DEFAULT_DISTANCE_M * sin(h_rad));
        int blocked = would_move_exit_geofence(&s->geofence, &s->pose, next_x, next_y);
        pthread_mutex_unlock(&s->ctrl_mutex);
        
        if (blocked) {
            snprintf(response, resp_len, "BLOCK\n");
            printf("[geofence] BACKWARD blocked at (%.4f, %.4f) -> (%.4f, %.4f)\n",
                   cur_x, cur_y, next_x, next_y);
        } else {
            int r = cmd_backward(s, duty);
            snprintf(response, resp_len, r ? "COLLISION\n" : "OK\n");
        }


    } else if (strcasecmp(cmd, "TURN") == 0) {
        float duty = 0.0f, deg = 0.0f;
        if (sscanf(line, "TURN %f %f", &duty, &deg) != 2) {
            snprintf(response, resp_len, "ERR TURN <duty> <deg>\n"); return; }
        cmd_turn(s, duty, deg);
        snprintf(response, resp_len, "OK\n");

    } else if (strcasecmp(cmd, "ROT") == 0) {
        float duty = 0.0f, deg = 0.0f;
        if (sscanf(line, "ROT %f %f", &duty, &deg) != 2) {
            snprintf(response, resp_len, "ERR ROT <duty> <deg>\n"); return; }
        int r = cmd_rot(s, duty, deg);
        snprintf(response, resp_len, r ? "COLLISION\n" : "OK\n");

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

    } else if (strcasecmp(cmd, "POSE") == 0) {
        cmd_pose(s, response, resp_len);

    } else if (strcasecmp(cmd, "POSN") == 0) {
        cmd_posn(s, response, resp_len);

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

    } else if (strcasecmp(cmd, "GEOFENCE") == 0) {
        char subcmd[32];
        if (sscanf(line, "GEOFENCE %31s", subcmd) != 1) {
            snprintf(response, resp_len, "ERR GEOFENCE <on|off|set|status>\n"); return; }
        
        if (strcasecmp(subcmd, "on") == 0) {
            pthread_mutex_lock(&s->ctrl_mutex);
            s->geofence.enabled = 1;
            pthread_mutex_unlock(&s->ctrl_mutex);
            snprintf(response, resp_len, "OK\n");
            printf("[geofence] enabled\n");
        } else if (strcasecmp(subcmd, "off") == 0) {
            pthread_mutex_lock(&s->ctrl_mutex);
            s->geofence.enabled = 0;
            pthread_mutex_unlock(&s->ctrl_mutex);
            snprintf(response, resp_len, "OK\n");
            printf("[geofence] disabled\n");
        } else if (strcasecmp(subcmd, "set") == 0) {
            double min_x, max_x, min_y, max_y;
            if (sscanf(line, "GEOFENCE set %lf %lf %lf %lf", &min_x, &max_x, &min_y, &max_y) != 4) {
                snprintf(response, resp_len, "ERR GEOFENCE set <min_x> <max_x> <min_y> <max_y>\n"); return; }
            pthread_mutex_lock(&s->ctrl_mutex);
            s->geofence.min_x_m = min_x;
            s->geofence.max_x_m = max_x;
            s->geofence.min_y_m = min_y;
            s->geofence.max_y_m = max_y;
            pthread_mutex_unlock(&s->ctrl_mutex);
            snprintf(response, resp_len, "OK\n");
            printf("[geofence] set bounds: x=[%.3f, %.3f] y=[%.3f, %.3f]\n",
                   min_x, max_x, min_y, max_y);
        } else if (strcasecmp(subcmd, "status") == 0) {
            pthread_mutex_lock(&s->ctrl_mutex);
            int enabled = s->geofence.enabled;
            double min_x = s->geofence.min_x_m;
            double max_x = s->geofence.max_x_m;
            double min_y = s->geofence.min_y_m;
            double max_y = s->geofence.max_y_m;
            double cur_x = s->pose.x_m;
            double cur_y = s->pose.y_m;
            int in_bounds = is_point_in_geofence(&s->geofence, cur_x, cur_y);
            pthread_mutex_unlock(&s->ctrl_mutex);
            snprintf(response, resp_len,
                "{\"enabled\":%d,\"min_x\":%.4f,\"max_x\":%.4f,\"min_y\":%.4f,\"max_y\":%.4f,\"cur_x\":%.4f,\"cur_y\":%.4f,\"in_bounds\":%d}\n",
                enabled, min_x, max_x, min_y, max_y, cur_x, cur_y, in_bounds);
        } else {
            snprintf(response, resp_len, "ERR GEOFENCE subcommand unknown: %s\n", subcmd);
        }

    } else if (strcasecmp(cmd, "COLLISION") == 0) {
        char subcmd[32];
        if (sscanf(line, "COLLISION %31s", subcmd) != 1) {
            snprintf(response, resp_len, "ERR COLLISION <on|off|threshold|rotstall|status>\n"); return; }

        if (strcasecmp(subcmd, "on") == 0) {
            pthread_mutex_lock(&s->ctrl_mutex);
            s->collision_detect = 1;
            pthread_mutex_unlock(&s->ctrl_mutex);
            snprintf(response, resp_len, "OK\n");
            printf("[collision] detection enabled\n");
        } else if (strcasecmp(subcmd, "off") == 0) {
            pthread_mutex_lock(&s->ctrl_mutex);
            s->collision_detect = 0;
            pthread_mutex_unlock(&s->ctrl_mutex);
            snprintf(response, resp_len, "OK\n");
            printf("[collision] detection disabled\n");
        } else if (strcasecmp(subcmd, "threshold") == 0) {
            float thresh = 0.0f;
            if (sscanf(line, "COLLISION threshold %f", &thresh) != 1) {
                snprintf(response, resp_len, "ERR COLLISION threshold <m/s^2>\n"); return; }
            pthread_mutex_lock(&s->ctrl_mutex);
            s->collision_threshold_ms2 = thresh;
            pthread_mutex_unlock(&s->ctrl_mutex);
            snprintf(response, resp_len, "OK\n");
            printf("[collision] threshold set to %.2f m/s^2\n", (double)thresh);
        } else if (strcasecmp(subcmd, "rotstall") == 0) {
            float min_rate_dps = 0.0f;
            float per_duty_dps = 0.0f;
            int hold_ms = 0;
            // New format: COLLISION rotstall <min_rate_dps> <per_duty_dps> <hold_ms>
            // Legacy format: COLLISION rotstall <rate_dps> <hold_ms>
            int parsed = sscanf(line, "COLLISION rotstall %f %f %d", &min_rate_dps, &per_duty_dps, &hold_ms);
            if (parsed == 2) {
                // Legacy fixed-threshold behavior
                hold_ms = (int)per_duty_dps;
                per_duty_dps = 0.0f;
            } else if (parsed != 3) {
                snprintf(response, resp_len,
                         "ERR COLLISION rotstall <min_rate_dps> <per_duty_dps> <hold_ms>\n"); return;
            }
            if (min_rate_dps <= 0.0f || per_duty_dps < 0.0f || hold_ms <= 0) {
                snprintf(response, resp_len, "ERR COLLISION rotstall expects positive values\n"); return; }
            pthread_mutex_lock(&s->ctrl_mutex);
            s->rot_stall_min_rate_dps = min_rate_dps;
            s->rot_stall_per_duty_dps = per_duty_dps;
            s->rot_stall_hold_us = hold_ms * 1000;
            pthread_mutex_unlock(&s->ctrl_mutex);
            snprintf(response, resp_len, "OK\n");
            printf("[collision] ROT stall params set: min=%.2f dps per_duty=%.2f hold=%d ms\n",
                   (double)min_rate_dps, (double)per_duty_dps, hold_ms);
        } else if (strcasecmp(subcmd, "status") == 0) {
            pthread_mutex_lock(&s->ctrl_mutex);
            int   det    = s->collision_detect;
            float thresh = s->collision_threshold_ms2;
            float stall_min_rate_dps = s->rot_stall_min_rate_dps;
            float stall_per_duty_dps = s->rot_stall_per_duty_dps;
            int   stall_hold_ms = s->rot_stall_hold_us / 1000;
            pthread_mutex_unlock(&s->ctrl_mutex);
            snprintf(response, resp_len,
                "{\"enabled\":%d,\"threshold_ms2\":%.2f,\"rot_stall_min_rate_dps\":%.2f,\"rot_stall_per_duty_dps\":%.2f,\"rot_stall_hold_ms\":%d}\n",
                det, (double)thresh, (double)stall_min_rate_dps, (double)stall_per_duty_dps, stall_hold_ms);
        } else {
            snprintf(response, resp_len, "ERR COLLISION subcommand unknown: %s\n", subcmd);
        }

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
    printf("  --collision        Enable collision detection at startup (default: off)\n");
    printf("  --collision-threshold T  Accel spike threshold above 1g in m/s^2 (default %.1f)\n",
           (double)DEFAULT_COLLISION_THRESHOLD_MS2);
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
    // Initialize geofence: disabled by default, no bounds
    s.geofence.enabled       = 0;
    s.geofence.min_x_m       = -1.0;
    s.geofence.max_x_m       =  1.0;
    s.geofence.min_y_m       = -1.0;
    s.geofence.max_y_m       =  1.0;
    // Collision detection: disabled by default
    s.collision_detect        = 0;
    s.collision_threshold_ms2 = DEFAULT_COLLISION_THRESHOLD_MS2;
    s.rot_stall_min_rate_dps  = (float)ROT_STALL_MIN_RATE_DPS;
    s.rot_stall_per_duty_dps  = (float)ROT_STALL_PER_DUTY_DPS;
    s.rot_stall_hold_us       = ROT_STALL_HOLD_US;

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--help"))        { print_usage(argv[0]); return 0; }
        else if (!strcmp(argv[i], "--port")         && i+1<argc) s.port                   = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--max-duty")     && i+1<argc) s.motor_ramp.max_duty    = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--accel-rate")   && i+1<argc) s.motor_ramp.accel_rate  = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--decel-rate")   && i+1<argc) s.motor_ramp.decel_rate  = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--heading-gain") && i+1<argc) s.straight.kp            = atof(argv[++i]);
        else if (!strcmp(argv[i], "--heading-tol")  && i+1<argc) s.straight.tolerance_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--watchdog-sec") && i+1<argc) s.watchdog_sec           = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--collision"))                  s.collision_detect        = 1;
        else if (!strcmp(argv[i], "--collision-threshold") && i+1<argc) s.collision_threshold_ms2 = (float)atof(argv[++i]);
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
           "gain=%.2f tol=%.1fdeg watchdog=%ds collision=%s(%.1f m/s^2)\n",
           s.port,
           (double)s.motor_ramp.max_duty, (double)s.motor_ramp.accel_rate,
           (double)s.motor_ramp.decel_rate,
           s.straight.kp, s.straight.tolerance_deg, s.watchdog_sec,
           s.collision_detect ? "on " : "off ", (double)s.collision_threshold_ms2);

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
