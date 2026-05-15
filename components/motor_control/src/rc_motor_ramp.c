/**
 * @file rc_motor_ramp.c
 * @brief Motor control with soft accel/decel ramping implementation.
 */

#include "rc_motor_ramp.h"
#include <rc/motor.h>
#include <rc/start_stop.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <unistd.h>

#define MOTOR_CH_M0 1
#define MOTOR_CH_M1 2
#define RAMP_THREAD_FREQ_HZ 100
#define RAMP_DT_SEC (1.0 / RAMP_THREAD_FREQ_HZ)  // 0.01 s

/**
 * @brief Background thread function: ramps motor duties toward targets.
 *
 * Runs at ~100 Hz. Each tick:
 *   1. Lock mutex
 *   2. Compute dt since last tick
 *   3. Ramp current toward target by at most (accel_rate or decel_rate) * dt
 *   4. Apply to rc_motor_set() with M1 polarity inversion
 *   5. Unlock
 *   6. Sleep to maintain 100 Hz cadence
 */
static void* motor_ramp_thread_func(void *arg)
{
    motor_ramp_t *ramp = (motor_ramp_t *)arg;

    while (ramp->running) {
        pthread_mutex_lock(&ramp->mutex);
        
        // Ramp left motor
        float delta_left = ramp->target_left - ramp->current_left;
        if (delta_left > 0.0f) {
            // Accelerating
            float max_step = ramp->accel_rate * (float)RAMP_DT_SEC;
            ramp->current_left = fminf(ramp->current_left + max_step, ramp->target_left);
        } else if (delta_left < 0.0f) {
            // Decelerating
            float max_step = ramp->decel_rate * (float)RAMP_DT_SEC;
            ramp->current_left = fmaxf(ramp->current_left - max_step, ramp->target_left);
        }
        
        // Ramp right motor
        float delta_right = ramp->target_right - ramp->current_right;
        if (delta_right > 0.0f) {
            float max_step = ramp->accel_rate * (float)RAMP_DT_SEC;
            ramp->current_right = fminf(ramp->current_right + max_step, ramp->target_right);
        } else if (delta_right < 0.0f) {
            float max_step = ramp->decel_rate * (float)RAMP_DT_SEC;
            ramp->current_right = fmaxf(ramp->current_right - max_step, ramp->target_right);
        }
        
        // Apply to motors: M1 polarity inverted so both motors drive forward together
        float m0_cmd = ramp->current_left;
        float m1_cmd = -ramp->current_right;  // Invert M1 for differential drive
        
        ramp->last_m0_cmd = m0_cmd;
        ramp->last_m1_cmd = m1_cmd;
        ramp->last_m0_ret = rc_motor_set(MOTOR_CH_M0, m0_cmd);
        ramp->last_m1_ret = rc_motor_set(MOTOR_CH_M1, m1_cmd);
        
        pthread_mutex_unlock(&ramp->mutex);
        
        // Sleep to maintain ~100 Hz cadence (10 ms)
        usleep(10000);  // 10 ms
    }
    
    // Cleanup: brake both motors
    rc_motor_brake(MOTOR_CH_M0);
    rc_motor_brake(MOTOR_CH_M1);
    rc_motor_cleanup();
    
    return NULL;
}

int motor_ramp_init(motor_ramp_t *ramp, float max_duty, float accel_rate, float decel_rate)
{
    if (!ramp) return -1;
    
    ramp->current_left = 0.0f;
    ramp->current_right = 0.0f;
    ramp->target_left = 0.0f;
    ramp->target_right = 0.0f;
    ramp->max_duty = fmaxf(0.1f, fminf(1.0f, max_duty));
    ramp->accel_rate = fmaxf(0.1f, accel_rate);
    ramp->decel_rate = fmaxf(0.1f, decel_rate);
    ramp->last_m0_cmd = 0.0f;
    ramp->last_m1_cmd = 0.0f;
    ramp->last_m0_ret = 0;
    ramp->last_m1_ret = 0;
    ramp->running = 1;

    // Initialize motor hardware here so failure is visible at startup
    if (rc_motor_init() != 0) {
        fprintf(stderr, "[motor_ramp] rc_motor_init failed\n");
        return -1;
    }

    if (pthread_mutex_init(&ramp->mutex, NULL) != 0) {
        fprintf(stderr, "[motor_ramp] pthread_mutex_init failed\n");
        rc_motor_cleanup();
        return -1;
    }

    if (pthread_create(&ramp->thread_id, NULL, motor_ramp_thread_func, ramp) != 0) {
        fprintf(stderr, "[motor_ramp] pthread_create failed\n");
        pthread_mutex_destroy(&ramp->mutex);
        rc_motor_cleanup();
        return -1;
    }

    return 0;
}

int motor_ramp_set_target(motor_ramp_t *ramp, float left, float right)
{
    if (!ramp) return -1;
    
    // Clamp to max_duty
    float clamped_left = fmaxf(-ramp->max_duty, fminf(ramp->max_duty, left));
    float clamped_right = fmaxf(-ramp->max_duty, fminf(ramp->max_duty, right));
    
    pthread_mutex_lock(&ramp->mutex);
    ramp->target_left = clamped_left;
    ramp->target_right = clamped_right;
    pthread_mutex_unlock(&ramp->mutex);
    
    return 0;
}

int motor_ramp_stop(motor_ramp_t *ramp)
{
    if (!ramp) return -1;
    
    pthread_mutex_lock(&ramp->mutex);
    ramp->target_left = 0.0f;
    ramp->target_right = 0.0f;
    pthread_mutex_unlock(&ramp->mutex);
    
    return 0;
}

int motor_ramp_brake(motor_ramp_t *ramp)
{
    if (!ramp) return -1;
    
    pthread_mutex_lock(&ramp->mutex);
    ramp->current_left = 0.0f;
    ramp->current_right = 0.0f;
    ramp->target_left = 0.0f;
    ramp->target_right = 0.0f;
    pthread_mutex_unlock(&ramp->mutex);
    
    // Immediately brake motors (this may race with thread, but both write 0)
    rc_motor_brake(MOTOR_CH_M0);
    rc_motor_brake(MOTOR_CH_M1);
    
    return 0;
}

int motor_ramp_get_current(motor_ramp_t *ramp, float *out_left, float *out_right)
{
    if (!ramp) return -1;
    
    pthread_mutex_lock(&ramp->mutex);
    if (out_left) *out_left = ramp->current_left;
    if (out_right) *out_right = ramp->current_right;
    pthread_mutex_unlock(&ramp->mutex);
    
    return 0;
}

int motor_ramp_get_driver_status(motor_ramp_t *ramp,
                                 float *m0_cmd,
                                 float *m1_cmd,
                                 int *m0_ret,
                                 int *m1_ret)
{
    if (!ramp) return -1;

    pthread_mutex_lock(&ramp->mutex);
    if (m0_cmd) *m0_cmd = ramp->last_m0_cmd;
    if (m1_cmd) *m1_cmd = ramp->last_m1_cmd;
    if (m0_ret) *m0_ret = ramp->last_m0_ret;
    if (m1_ret) *m1_ret = ramp->last_m1_ret;
    pthread_mutex_unlock(&ramp->mutex);

    return 0;
}

int motor_ramp_set_rates(motor_ramp_t *ramp, float accel_rate, float decel_rate)
{
    if (!ramp) return -1;
    
    pthread_mutex_lock(&ramp->mutex);
    ramp->accel_rate = fmaxf(0.1f, accel_rate);
    ramp->decel_rate = fmaxf(0.1f, decel_rate);
    pthread_mutex_unlock(&ramp->mutex);
    
    return 0;
}

int motor_ramp_cleanup(motor_ramp_t *ramp)
{
    if (!ramp) return -1;
    
    // Signal thread to stop
    pthread_mutex_lock(&ramp->mutex);
    ramp->running = 0;
    pthread_mutex_unlock(&ramp->mutex);
    
    // Wait for thread to exit (with timeout guard)
    void *thread_result;
    pthread_join(ramp->thread_id, &thread_result);
    
    pthread_mutex_destroy(&ramp->mutex);
    
    return 0;
}
