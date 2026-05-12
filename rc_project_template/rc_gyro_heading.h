/**
 * @file rc_gyro_heading.h
 * @brief Gyroscope + magnetometer heading tracker.
 *
 * A 100 Hz background thread integrates gyro Z for fast heading updates.
 * Every 100 ms the magnetometer is also read; the compass bearing can be used
 * to reseed the integrated heading whenever motors are stopped, preventing
 * gyro-drift accumulation across multiple start/stop cycles.
 */

#ifndef RC_GYRO_HEADING_H
#define RC_GYRO_HEADING_H

#include <pthread.h>

/**
 * @struct gyro_heading_t
 * @brief Gyroscope + magnetometer heading tracker state.
 *
 * All members are protected by the internal mutex.
 */
typedef struct {
    double heading_deg;         ///< Integrated gyro heading [-180, +180]
    double last_gyro_z_dps;     ///< Last gyro Z rate in deg/s
    double compass_deg;         ///< Latest magnetometer bearing [-180, +180]
    int    compass_valid;        ///< 1 once the first mag reading succeeds
    double mag_x_ut;            ///< Latest raw magnetometer X field (uT)
    double mag_y_ut;            ///< Latest raw magnetometer Y field (uT)
    double mag_z_ut;            ///< Latest raw magnetometer Z field (uT)
    int    mag_sample_count;    ///< Successful magnetometer reads since startup
    int    mag_error_count;     ///< Failed magnetometer reads since startup

    int running;                ///< 1 if thread is active
    pthread_t thread_id;
    pthread_mutex_t mutex;
} gyro_heading_t;

/**
 * @brief Initialize gyro heading tracker and start background thread.
 *
 * @param heading Pointer to gyro_heading_t to initialize
 * @return 0 on success, -1 on error
 */
int gyro_heading_init(gyro_heading_t *heading);

/** @brief Reset integrated heading to 0. Does not affect compass_deg. */
int gyro_heading_reset(gyro_heading_t *heading);

/** @brief Get current integrated gyro heading in degrees [-180, +180]. */
double gyro_heading_get_deg(gyro_heading_t *heading);

/** @brief Get latest gyro Z rate in deg/s. */
double gyro_heading_get_rate_dps(gyro_heading_t *heading);

/**
 * @brief Get latest magnetometer bearing in degrees [-180, +180].
 *
 * Returns 0.0 if no valid mag reading has been obtained yet; check
 * gyro_heading_compass_valid() first.
 */
double gyro_heading_get_compass_deg(gyro_heading_t *heading);

/** @brief Returns 1 if at least one valid magnetometer reading has been obtained. */
int gyro_heading_compass_valid(gyro_heading_t *heading);

/** @brief Copy latest raw magnetometer field values in uT. */
int gyro_heading_get_mag_raw(gyro_heading_t *heading, double *mag_x, double *mag_y, double *mag_z);

/** @brief Get total successful and failed magnetometer read counts. */
int gyro_heading_get_mag_counters(gyro_heading_t *heading, int *sample_count, int *error_count);

/**
 * @brief Reseed the integrated gyro heading from the latest magnetometer reading.
 *
 * Sets heading_deg = compass_deg.  Call this when motors are at rest
 * (no EMI from the H-bridge) to correct accumulated gyro drift.
 *
 * @return 0 on success, -1 if compass_valid is 0.
 */
int gyro_heading_reseed_from_compass(gyro_heading_t *heading);

/** @brief Stop background thread and clean up. Safe to call multiple times. */
int gyro_heading_cleanup(gyro_heading_t *heading);

#endif // RC_GYRO_HEADING_H
