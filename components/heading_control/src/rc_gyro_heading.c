/**
 * @file rc_gyro_heading.c
 * @brief Gyroscope + magnetometer heading tracker implementation.
 */

#include "rc_gyro_heading.h"
#include <rc/mpu.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#define I2C_BUS              2
#define HEADING_HZ           100           ///< Gyro integration rate
#define HEADING_DT           (1.0 / HEADING_HZ)
#define MAG_DECIMATE         10            ///< Read mag every 10 ticks = 100 ms
#define MPU_READ_RETRIES     5
#define MAG_REINIT_THRESHOLD 3

/** Wrap angle to [-180, +180]. */
static double wrap_angle(double deg)
{
    while (deg >  180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
}

/**
 * Convert magnetometer plus accelerometer readings to a tilt-compensated
 * bearing in degrees.
 */
static double mag_to_deg(double acc_x, double acc_y, double acc_z,
                         double mag_x, double mag_y, double mag_z)
{
    double roll  = atan2(acc_y, acc_z);
    double pitch = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z));

    double xh = mag_x * cos(pitch) + mag_z * sin(pitch);
    double yh = mag_x * sin(roll) * sin(pitch)
              + mag_y * cos(roll)
              - mag_z * sin(roll) * cos(pitch);

    double deg = -atan2(yh, xh) * (180.0 / M_PI);
    return wrap_angle(deg);
}

static int initialize_mpu(rc_mpu_data_t *mpu, rc_mpu_config_t *conf)
{
    *conf = rc_mpu_default_config();
    conf->i2c_bus = I2C_BUS;
    conf->enable_magnetometer = 1;

    if (rc_mpu_initialize(mpu, *conf) < 0) {
        fprintf(stderr, "[gyro_heading] rc_mpu_initialize failed\n");
        rc_mpu_power_off();
        return -1;
    }

    // Allow the AK8963 time to settle and start producing fresh samples.
    usleep(200000);
    return 0;
}

static void* gyro_heading_thread_func(void *arg)
{
    gyro_heading_t *h = (gyro_heading_t *)arg;
    rc_mpu_data_t   mpu;
    rc_mpu_config_t conf;
    int tick = 0;
    int mag_failures = 0;

    if (initialize_mpu(&mpu, &conf) < 0) {
        return NULL;
    }

    while (h->running) {
        // --- Gyro integration (100 Hz) ---
        int ok = 0;
        double gyro_z_dps = 0.0;
        double acc_x = 0.0;
        double acc_y = 0.0;
        double acc_z = 0.0;
        for (int i = 0; i < MPU_READ_RETRIES; i++) {
            if (rc_mpu_read_accel(&mpu) != 0) {
                usleep(2000);
                continue;
            }
            if (rc_mpu_read_gyro(&mpu) == 0) {
                acc_x = mpu.accel[0];
                acc_y = mpu.accel[1];
                acc_z = mpu.accel[2];
                gyro_z_dps = mpu.gyro[2];
                ok = 1;
                break;
            }
            usleep(2000);
        }
        if (ok) {
            pthread_mutex_lock(&h->mutex);
            h->last_gyro_z_dps  = gyro_z_dps;
            h->heading_deg     += gyro_z_dps * HEADING_DT;
            h->heading_deg      = wrap_angle(h->heading_deg);
            pthread_mutex_unlock(&h->mutex);
        }

        // --- Magnetometer sample (every MAG_DECIMATE ticks = 100 ms) ---
        if (++tick >= MAG_DECIMATE) {
            tick = 0;
            if (rc_mpu_read_mag(&mpu) == 0) {
                pthread_mutex_lock(&h->mutex);
                h->mag_x_ut      = mpu.mag[0];
                h->mag_y_ut      = mpu.mag[1];
                h->mag_z_ut      = mpu.mag[2];
                h->compass_deg   = mag_to_deg(acc_x, acc_y, acc_z,
                                               mpu.mag[0], mpu.mag[1], mpu.mag[2]);
                h->compass_valid = 1;
                h->mag_sample_count++;
                pthread_mutex_unlock(&h->mutex);
                mag_failures = 0;
            } else if (++mag_failures >= MAG_REINIT_THRESHOLD) {
                fprintf(stderr, "[gyro_heading] magnetometer read failed repeatedly, reinitializing MPU\n");
                pthread_mutex_lock(&h->mutex);
                h->compass_valid = 0;
                h->mag_error_count += mag_failures;
                pthread_mutex_unlock(&h->mutex);
                rc_mpu_power_off();
                if (initialize_mpu(&mpu, &conf) < 0) {
                    return NULL;
                }
                mag_failures = 0;
            } else {
                pthread_mutex_lock(&h->mutex);
                h->mag_error_count++;
                pthread_mutex_unlock(&h->mutex);
            }
        }
        usleep(10000);  // 10 ms -> 100 Hz
    }

    rc_mpu_power_off();
    return NULL;
}

int gyro_heading_init(gyro_heading_t *heading)
{
    if (!heading) return -1;

    heading->heading_deg     = 0.0;
    heading->last_gyro_z_dps = 0.0;
    heading->compass_deg     = 0.0;
    heading->compass_valid   = 0;
    heading->mag_x_ut        = 0.0;
    heading->mag_y_ut        = 0.0;
    heading->mag_z_ut        = 0.0;
    heading->mag_sample_count = 0;
    heading->mag_error_count  = 0;
    heading->running         = 1;

    if (pthread_mutex_init(&heading->mutex, NULL) != 0) {
        fprintf(stderr, "[gyro_heading] pthread_mutex_init failed\n");
        return -1;
    }

    if (pthread_create(&heading->thread_id, NULL, gyro_heading_thread_func, heading) != 0) {
        fprintf(stderr, "[gyro_heading] pthread_create failed\n");
        pthread_mutex_destroy(&heading->mutex);
        return -1;
    }

    return 0;
}

int gyro_heading_reset(gyro_heading_t *heading)
{
    if (!heading) return -1;
    pthread_mutex_lock(&heading->mutex);
    heading->heading_deg = 0.0;
    pthread_mutex_unlock(&heading->mutex);
    return 0;
}

double gyro_heading_get_deg(gyro_heading_t *heading)
{
    if (!heading) return 0.0;
    pthread_mutex_lock(&heading->mutex);
    double deg = heading->heading_deg;
    pthread_mutex_unlock(&heading->mutex);
    return deg;
}

double gyro_heading_get_rate_dps(gyro_heading_t *heading)
{
    if (!heading) return 0.0;
    pthread_mutex_lock(&heading->mutex);
    double rate = heading->last_gyro_z_dps;
    pthread_mutex_unlock(&heading->mutex);
    return rate;
}

double gyro_heading_get_compass_deg(gyro_heading_t *heading)
{
    if (!heading) return 0.0;
    pthread_mutex_lock(&heading->mutex);
    double deg = heading->compass_deg;
    pthread_mutex_unlock(&heading->mutex);
    return deg;
}

int gyro_heading_compass_valid(gyro_heading_t *heading)
{
    if (!heading) return 0;
    pthread_mutex_lock(&heading->mutex);
    int v = heading->compass_valid;
    pthread_mutex_unlock(&heading->mutex);
    return v;
}

int gyro_heading_get_mag_raw(gyro_heading_t *heading, double *mag_x, double *mag_y, double *mag_z)
{
    if (!heading) return -1;
    pthread_mutex_lock(&heading->mutex);
    if (mag_x) *mag_x = heading->mag_x_ut;
    if (mag_y) *mag_y = heading->mag_y_ut;
    if (mag_z) *mag_z = heading->mag_z_ut;
    pthread_mutex_unlock(&heading->mutex);
    return 0;
}

int gyro_heading_get_mag_counters(gyro_heading_t *heading, int *sample_count, int *error_count)
{
    if (!heading) return -1;
    pthread_mutex_lock(&heading->mutex);
    if (sample_count) *sample_count = heading->mag_sample_count;
    if (error_count) *error_count = heading->mag_error_count;
    pthread_mutex_unlock(&heading->mutex);
    return 0;
}

int gyro_heading_reseed_from_compass(gyro_heading_t *heading)
{
    if (!heading) return -1;
    pthread_mutex_lock(&heading->mutex);
    if (!heading->compass_valid) {
        pthread_mutex_unlock(&heading->mutex);
        return -1;
    }
    heading->heading_deg = heading->compass_deg;
    pthread_mutex_unlock(&heading->mutex);
    return 0;
}

int gyro_heading_cleanup(gyro_heading_t *heading)
{
    if (!heading) return -1;
    
    // Signal thread to stop
    pthread_mutex_lock(&heading->mutex);
    heading->running = 0;
    pthread_mutex_unlock(&heading->mutex);
    
    // Wait for thread to exit
    void *thread_result;
    pthread_join(heading->thread_id, &thread_result);
    
    pthread_mutex_destroy(&heading->mutex);
    
    return 0;
}
