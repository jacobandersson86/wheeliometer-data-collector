#pragma once

#include <stdint.h>
#include <stddef.h>

// IMU sample data structure (one sample from FIFO)
struct imu_sample_t {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} __attribute__((packed));

// Batch of samples with base timestamp for reconstruction
struct imu_batch_t {
    uint64_t base_timestamp_us;  // Timestamp of first sample in microseconds
    uint16_t sample_count;       // Number of samples in this batch
    imu_sample_t samples[40];    // Up to 40 samples (fits in FIFO)
};

// Configuration structure
struct imu_sampler_config_t {
    uint8_t accel_fsr;  // Accelerometer full-scale range (0-3)
    uint8_t gyro_fsr;   // Gyroscope full-scale range (0-3)
    uint8_t odr;        // Output data rate
    uint8_t int_pin;    // GPIO pin for interrupt
    size_t queue_depth; // Depth of the FreeRTOS queue
};

/**
 * Initialize the IMU sampler
 * @param config Configuration structure
 * @return true if successful, false otherwise
 */
bool imu_sampler_init(const imu_sampler_config_t* config);

/**
 * Start sampling (interrupt-driven)
 * @return true if successful, false otherwise
 */
bool imu_sampler_start(void);

/**
 * Stop sampling
 * @return true if successful, false otherwise
 */
bool imu_sampler_stop(void);

/**
 * Check if sampling is active
 * @return true if sampling, false otherwise
 */
bool imu_sampler_is_running(void);

/**
 * Get the queue handle for receiving batches
 * @return FreeRTOS queue handle (QueueHandle_t)
 */
void* imu_sampler_get_queue(void);

/**
 * Get statistics
 */
struct imu_sampler_stats_t {
    uint64_t total_samples;
    uint32_t fifo_overflows;
    uint32_t batches_sent;
    uint32_t queue_full_errors;
};

void imu_sampler_get_stats(imu_sampler_stats_t* stats);
void imu_sampler_reset_stats(void);
