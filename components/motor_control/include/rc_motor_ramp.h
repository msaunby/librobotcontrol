/**
 * @file rc_motor_ramp.h
 * @brief Motor control with soft accel/decel ramping.
 *
 * Provides a 100 Hz background thread that enforces acceleration and deceleration
 * rate limits on motor commands. Supports independent left/right motors or paired
 * mode with M1 polarity inversion for differential drive.
 */

#ifndef RC_MOTOR_RAMP_H
#define RC_MOTOR_RAMP_H

#include <pthread.h>

/**
 * @struct motor_ramp_t
 * @brief Motor ramp controller state.
 *
 * Contains current and target duty cycles, ramp rates, and threading primitives.
 * All members are protected by the internal mutex.
 */
typedef struct {
    float current_left;         ///< Current left motor duty [-1, +1]
    float current_right;        ///< Current right motor duty [-1, +1]
    float target_left;          ///< Target left motor duty (set by commands)
    float target_right;         ///< Target right motor duty
    float accel_rate;           ///< Max accel in duty/sec (default 0.5)
    float decel_rate;           ///< Max decel in duty/sec (default 1.0)
    float max_duty;             ///< Max absolute duty (default 0.6)
    float last_m0_cmd;          ///< Last duty sent to motor channel M0
    float last_m1_cmd;          ///< Last duty sent to motor channel M1
    int   last_m0_ret;          ///< Last rc_motor_set return code for M0
    int   last_m1_ret;          ///< Last rc_motor_set return code for M1
    
    int running;                ///< 1 if thread is active, 0 otherwise
    pthread_t thread_id;        ///< Background thread ID
    pthread_mutex_t mutex;      ///< Protects all state above
} motor_ramp_t;

/**
 * @brief Initialize motor ramp controller and start background thread.
 *
 * @param ramp Pointer to motor_ramp_t to initialize
 * @param max_duty Maximum absolute duty cycle (default 0.6)
 * @param accel_rate Acceleration rate in duty/sec (default 0.5 = 2s ramp)
 * @param decel_rate Deceleration rate in duty/sec (default 1.0 = 1s ramp)
 * @return 0 on success, -1 on error
 */
int motor_ramp_init(motor_ramp_t *ramp, float max_duty, float accel_rate, float decel_rate);

/**
 * @brief Set target duty for left and right motors.
 *
 * @param ramp Pointer to motor_ramp_t
 * @param left Target left duty [-1, +1], clamped to [-max_duty, +max_duty]
 * @param right Target right duty [-1, +1], clamped to [-max_duty, +max_duty]
 * @return 0 on success
 */
int motor_ramp_set_target(motor_ramp_t *ramp, float left, float right);

/**
 * @brief Immediately stop both motors with no ramp (sets target to 0, current stays ramped).
 *
 * Ramping continues in background thread; call motor_ramp_brake() for immediate stop.
 *
 * @param ramp Pointer to motor_ramp_t
 * @return 0 on success
 */
int motor_ramp_stop(motor_ramp_t *ramp);

/**
 * @brief Emergency brake: immediately stop motors without ramping.
 *
 * Calls rc_motor_brake() on both channels and resets state to 0.
 *
 * @param ramp Pointer to motor_ramp_t
 * @return 0 on success
 */
int motor_ramp_brake(motor_ramp_t *ramp);

/**
 * @brief Get current duty cycles (non-blocking read).
 *
 * @param ramp Pointer to motor_ramp_t
 * @param out_left Pointer to receive current left duty, or NULL
 * @param out_right Pointer to receive current right duty, or NULL
 * @return 0 on success
 */
int motor_ramp_get_current(motor_ramp_t *ramp, float *out_left, float *out_right);

/** @brief Get last commanded motor outputs and rc_motor_set return codes. */
int motor_ramp_get_driver_status(motor_ramp_t *ramp,
                                 float *m0_cmd,
                                 float *m1_cmd,
                                 int *m0_ret,
                                 int *m1_ret);

/**
 * @brief Update ramp rates on the fly.
 *
 * @param ramp Pointer to motor_ramp_t
 * @param accel_rate New acceleration rate (duty/sec)
 * @param decel_rate New deceleration rate (duty/sec)
 * @return 0 on success
 */
int motor_ramp_set_rates(motor_ramp_t *ramp, float accel_rate, float decel_rate);

/**
 * @brief Stop background thread and clean up resources.
 *
 * Brakes motors before exiting. Safe to call multiple times.
 *
 * @param ramp Pointer to motor_ramp_t
 * @return 0 on success
 */
int motor_ramp_cleanup(motor_ramp_t *ramp);

#endif // RC_MOTOR_RAMP_H
