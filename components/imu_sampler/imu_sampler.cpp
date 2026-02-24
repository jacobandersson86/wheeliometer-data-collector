#include "imu_sampler.hpp"
#include <M5Unified.h>
#include <utility/imu/MPU6886_Class.hpp>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <algorithm>

static const char* TAG = "IMU_SAMPLER";
static constexpr uint16_t BATCH_SAMPLE_CAPACITY = 60;
static constexpr uint32_t IDLE_POLL_DELAY_MS = 2;
static constexpr uint32_t BUSY_POLL_DELAY_MS = 1;
static constexpr int64_t SOFT_LOCK_WINDOW_US = 2000;
static constexpr int64_t HARD_RESYNC_THRESHOLD_US = 20000;
static constexpr int64_t MAX_SLEW_CORRECTION_US = 200;

static uint32_t odr_to_period_us(uint8_t odr) {
    switch (odr) {
        case 0: return 1000;     // 1 kHz
        case 1: return 2000;     // 500 Hz
        case 3: return 4000;     // 250 Hz
        case 4: return 5000;     // 200 Hz
        case 7: return 8000;     // 125 Hz
        case 9: return 10000;    // 100 Hz
        case 19: return 20000;   // 50 Hz
        case 99: return 100000;  // 10 Hz
        default: return 1000;
    }
}

// Global state
static struct {
    bool initialized;
    bool running;
    m5::MPU6886_Class* mpu;
    gpio_num_t int_pin;
    QueueHandle_t queue;
    TaskHandle_t task_handle;
    imu_sampler_config_t config;
    imu_sampler_stats_t stats;
    volatile bool data_ready_flag;
} g_state{};

// Forward declarations
static void imu_sampler_task(void* pvParameters);

bool imu_sampler_init(const imu_sampler_config_t* config) {
    if (g_state.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return true;
    }

    if (!config) {
        ESP_LOGE(TAG, "Invalid config");
        return false;
    }

    // Store config
    g_state.config = *config;
    g_state.int_pin = (gpio_num_t)config->int_pin;

    // Get MPU6886 instance from M5Unified
    g_state.mpu = static_cast<m5::MPU6886_Class*>(M5.Imu.getImuInstancePtr(0));
    if (!g_state.mpu) {
        ESP_LOGE(TAG, "Failed to get MPU6886 instance");
        return false;
    }

    // Check IMU type
    if (M5.Imu.getType() != m5::imu_t::imu_mpu6886) {
        ESP_LOGE(TAG, "IMU is not MPU6886");
        return false;
    }

    // Note: MPU6886 is already configured by M5Unified with default ranges:
    // ±8G for accelerometer and ±2000 DPS for gyroscope
    // These are set in MPU6886_Class::begin() and cannot be changed externally

    // Note: On M5StickC Plus, the MPU6886 INT pin may not be connected.
    // GPIO 35 is also input-only on ESP32 (no internal pullup available).
    // We'll use polling mode instead of interrupts.
    ESP_LOGI(TAG, "Using polling mode (adaptive interval) - INT pin not used");

    // Create queue for batches
    g_state.queue = xQueueCreate(config->queue_depth, sizeof(imu_batch_t));
    if (!g_state.queue) {
        ESP_LOGE(TAG, "Failed to create queue");
        return false;
    }

    // Create high-priority task for reading FIFO
    BaseType_t ret = xTaskCreatePinnedToCore(
        imu_sampler_task,
        "imu_sampler",
        4096,  // Stack size
        NULL,
        20,    // High priority (configMAX_PRIORITIES - 3)
        &g_state.task_handle,
        1      // Core 1
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        vQueueDelete(g_state.queue);
        return false;
    }

    g_state.initialized = true;
    ESP_LOGI(TAG, "Initialized successfully");

    return true;
}

bool imu_sampler_start(void) {
    if (!g_state.initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return false;
    }

    if (g_state.running) {
        ESP_LOGW(TAG, "Already running");
        return true;
    }

    // Reset statistics
    memset(&g_state.stats, 0, sizeof(g_state.stats));

    // Enable FIFO with 1kHz sampling
    g_state.mpu->enableFIFO((m5::MPU6886_Class::Fodr)g_state.config.odr);

    // Small delay to let FIFO initialize
    vTaskDelay(pdMS_TO_TICKS(10));

    // Verify FIFO is enabled
    uint8_t user_ctrl = g_state.mpu->readRegister8(m5::MPU6886_Class::REG_USER_CTRL);
    uint8_t fifo_en = g_state.mpu->readRegister8(m5::MPU6886_Class::REG_FIFO_EN);
    ESP_LOGI(TAG, "FIFO enabled: USER_CTRL=0x%02X FIFO_EN=0x%02X", user_ctrl, fifo_en);

    g_state.running = true;
    ESP_LOGI(TAG, "Sampling started");

    return true;
}

bool imu_sampler_stop(void) {
    if (!g_state.running) {
        return true;
    }

    g_state.running = false;
    ESP_LOGI(TAG, "Sampling stopped");

    return true;
}

bool imu_sampler_is_running(void) {
    return g_state.running;
}

void* imu_sampler_get_queue(void) {
    return g_state.queue;
}

void imu_sampler_get_stats(imu_sampler_stats_t* stats) {
    if (stats) {
        *stats = g_state.stats;
    }
}

void imu_sampler_reset_stats(void) {
    memset(&g_state.stats, 0, sizeof(g_state.stats));
}

// High-priority task that reads FIFO data
static void imu_sampler_task(void* pvParameters) {
    (void)pvParameters;

    imu_batch_t batch;
    uint8_t fifo_count_buf[2];
    uint8_t fifo_data[14];  // One FIFO entry: 14 bytes
    const uint32_t sample_period_us = odr_to_period_us(g_state.config.odr);
    bool timestamp_lock_initialized = false;
    uint64_t next_expected_timestamp_us = 0;

    ESP_LOGI(TAG, "Sampler task started");

    while (true) {
        if (!g_state.running) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        bool drained_any_samples = false;

        while (g_state.running) {
            if (!g_state.mpu->readRegister(m5::MPU6886_Class::REG_FIFO_COUNTH, fifo_count_buf, 2)) {
                break;
            }

            uint16_t fifo_count = (fifo_count_buf[0] << 8) | fifo_count_buf[1];

            if (fifo_count == 0) {
                break;
            }

            if (fifo_count > 700) {
                ESP_LOGW(TAG, "FIFO near full: %u bytes", fifo_count);
            }

            if (fifo_count > 2048 || fifo_count == 0xFFFF) {
                g_state.stats.fifo_overflows++;
                ESP_LOGW(TAG, "FIFO count corrupted: %u, resetting", fifo_count);

                g_state.mpu->writeRegister8(m5::MPU6886_Class::REG_USER_CTRL, 0x00);
                vTaskDelay(pdMS_TO_TICKS(5));
                g_state.mpu->enableFIFO((m5::MPU6886_Class::Fodr)g_state.config.odr);
                break;
            }

            uint16_t available_samples = fifo_count / 14;
            if (available_samples == 0) {
                break;
            }

            uint16_t num_samples = available_samples;
            if (num_samples > BATCH_SAMPLE_CAPACITY) {
                num_samples = BATCH_SAMPLE_CAPACITY;
            }

            const uint64_t capture_time_us = esp_timer_get_time();
            batch.sample_count = num_samples;

            for (uint16_t i = 0; i < num_samples; i++) {
                if (!g_state.mpu->readRegister(m5::MPU6886_Class::REG_FIFO_R_W, fifo_data, 14)) {
                    ESP_LOGE(TAG, "Failed to read FIFO data at sample %u", i);
                    batch.sample_count = i;
                    break;
                }

                batch.samples[i].accel_x = (int16_t)((fifo_data[0] << 8) | fifo_data[1]);
                batch.samples[i].accel_y = (int16_t)((fifo_data[2] << 8) | fifo_data[3]);
                batch.samples[i].accel_z = (int16_t)((fifo_data[4] << 8) | fifo_data[5]);
                batch.samples[i].temp    = (int16_t)((fifo_data[6] << 8) | fifo_data[7]);
                batch.samples[i].gyro_x  = (int16_t)((fifo_data[8] << 8) | fifo_data[9]);
                batch.samples[i].gyro_y  = (int16_t)((fifo_data[10] << 8) | fifo_data[11]);
                batch.samples[i].gyro_z  = (int16_t)((fifo_data[12] << 8) | fifo_data[13]);
            }

            if (batch.sample_count == 0) {
                break;
            }

            drained_any_samples = true;

            const uint64_t measured_base_timestamp_us =
                capture_time_us - (uint64_t)(batch.sample_count - 1) * sample_period_us;

            if (!timestamp_lock_initialized) {
                batch.base_timestamp_us = measured_base_timestamp_us;
                next_expected_timestamp_us = measured_base_timestamp_us + (uint64_t)batch.sample_count * sample_period_us;
                timestamp_lock_initialized = true;
            } else {
                const uint64_t predicted_base_timestamp_us = next_expected_timestamp_us;
                const int64_t phase_error_us =
                    (int64_t)measured_base_timestamp_us - (int64_t)predicted_base_timestamp_us;

                if (std::llabs(phase_error_us) >= HARD_RESYNC_THRESHOLD_US) {
                    batch.base_timestamp_us = measured_base_timestamp_us;
                } else if (std::llabs(phase_error_us) <= SOFT_LOCK_WINDOW_US) {
                    int64_t correction_us = std::clamp(
                        phase_error_us,
                        -MAX_SLEW_CORRECTION_US,
                        MAX_SLEW_CORRECTION_US
                    );
                    batch.base_timestamp_us = (uint64_t)((int64_t)predicted_base_timestamp_us + correction_us);
                } else {
                    batch.base_timestamp_us = predicted_base_timestamp_us;
                }

                next_expected_timestamp_us =
                    batch.base_timestamp_us + (uint64_t)batch.sample_count * sample_period_us;
            }

            g_state.stats.total_samples += batch.sample_count;
            g_state.stats.batches_sent++;

            if (xQueueSend(g_state.queue, &batch, 0) != pdTRUE) {
                g_state.stats.queue_full_errors++;
                ESP_LOGW(TAG, "Queue full, dropping batch");
            }

            if (available_samples <= BATCH_SAMPLE_CAPACITY) {
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(drained_any_samples ? BUSY_POLL_DELAY_MS : IDLE_POLL_DELAY_MS));
    }
}
