#pragma once

#include <cstdint>

/**
 * Initialize the IMU consumer (running average calculator and printer)
 * @param queue_handle Queue handle from imu_sampler_get_queue()
 * @return true if successful
 */
bool imu_consumer_init(void* queue_handle);

/**
 * Get current running averages
 */
struct imu_averages_t {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;
    uint64_t sample_count;
};

void imu_consumer_get_averages(imu_averages_t* avg);
void imu_consumer_reset_averages(void);
