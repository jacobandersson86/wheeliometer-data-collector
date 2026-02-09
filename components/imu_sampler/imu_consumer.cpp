#include "imu_consumer.hpp"
#include "imu_sampler.hpp"
#include <M5Unified.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <terminal.hpp>

static const char* TAG = "IMU_CONSUMER";

// Conversion factors (based on ±8G and ±2000DPS defaults)
static const float ACCEL_RES = 8.0f / 32768.0f;   // G per LSB
static const float GYRO_RES = 2000.0f / 32768.0f; // DPS per LSB
static const float TEMP_RES = 1.0f / 326.8f;      // Degrees C per LSB
static const float TEMP_OFFSET = 25.0f;           // Temperature offset

// Global state
static struct {
    bool initialized;
    QueueHandle_t queue;
    TaskHandle_t task_handle;
    imu_averages_t averages;
    uint64_t last_print_time;
} g_consumer = {0};

// Forward declaration
static void imu_consumer_task(void* pvParameters);

bool imu_consumer_init(void* queue_handle) {
    if (g_consumer.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return true;
    }

    if (!queue_handle) {
        ESP_LOGE(TAG, "Invalid queue handle");
        return false;
    }

    g_consumer.queue = (QueueHandle_t)queue_handle;

    // Reset averages
    memset(&g_consumer.averages, 0, sizeof(g_consumer.averages));
    g_consumer.last_print_time = 0;

    // Create consumer task (normal priority)
    BaseType_t ret = xTaskCreatePinnedToCore(
        imu_consumer_task,
        "imu_consumer",
        4096,  // Stack size
        NULL,
        5,     // Normal priority
        &g_consumer.task_handle,
        0      // Core 0
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        return false;
    }

    g_consumer.initialized = true;
    ESP_LOGI(TAG, "Consumer initialized");

    return true;
}

void imu_consumer_get_averages(imu_averages_t* avg) {
    if (avg) {
        *avg = g_consumer.averages;
    }
}

void imu_consumer_reset_averages(void) {
    memset(&g_consumer.averages, 0, sizeof(g_consumer.averages));
}

// Consumer task - processes batches from queue
static void imu_consumer_task(void* pvParameters) {
    (void)pvParameters;

    imu_batch_t batch;
    char msg_buf[128];

    ESP_LOGI(TAG, "Consumer task started");

    while (true) {
        // Wait for batch from queue (blocking, 1 second timeout)
        if (xQueueReceive(g_consumer.queue, &batch, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // Process each sample in the batch
            for (uint16_t i = 0; i < batch.sample_count; i++) {
                const imu_sample_t* sample = &batch.samples[i];

                // Convert to physical units
                float ax = sample->accel_x * ACCEL_RES;
                float ay = sample->accel_y * ACCEL_RES;
                float az = sample->accel_z * ACCEL_RES;
                float gx = sample->gyro_x * GYRO_RES;
                float gy = sample->gyro_y * GYRO_RES;
                float gz = sample->gyro_z * GYRO_RES;
                float temp = TEMP_OFFSET + (sample->temp * TEMP_RES);

                // Update running averages using incremental mean formula
                // new_avg = old_avg + (new_value - old_avg) / (count + 1)
                uint64_t n = g_consumer.averages.sample_count;

                g_consumer.averages.accel_x = g_consumer.averages.accel_x + (ax - g_consumer.averages.accel_x) / (n + 1);
                g_consumer.averages.accel_y = g_consumer.averages.accel_y + (ay - g_consumer.averages.accel_y) / (n + 1);
                g_consumer.averages.accel_z = g_consumer.averages.accel_z + (az - g_consumer.averages.accel_z) / (n + 1);
                g_consumer.averages.gyro_x = g_consumer.averages.gyro_x + (gx - g_consumer.averages.gyro_x) / (n + 1);
                g_consumer.averages.gyro_y = g_consumer.averages.gyro_y + (gy - g_consumer.averages.gyro_y) / (n + 1);
                g_consumer.averages.gyro_z = g_consumer.averages.gyro_z + (gz - g_consumer.averages.gyro_z) / (n + 1);
                g_consumer.averages.temp = g_consumer.averages.temp + (temp - g_consumer.averages.temp) / (n + 1);

                g_consumer.averages.sample_count++;
            }

            // Print averages every 1 second
            uint64_t now = esp_timer_get_time();
            if (now - g_consumer.last_print_time >= 1000000) {  // 1 second
                snprintf(msg_buf, sizeof(msg_buf),
                    "IMU Avg (n=%llu): Acc[%.2f,%.2f,%.2f]g Gyro[%.1f,%.1f,%.1f]dps T=%.1fC",
                    g_consumer.averages.sample_count,
                    g_consumer.averages.accel_x,
                    g_consumer.averages.accel_y,
                    g_consumer.averages.accel_z,
                    g_consumer.averages.gyro_x,
                    g_consumer.averages.gyro_y,
                    g_consumer.averages.gyro_z,
                    g_consumer.averages.temp
                );

                terminal_write(msg_buf);
                ESP_LOGI(TAG, "%s", msg_buf);

                g_consumer.last_print_time = now;
            }
        }
    }
}
