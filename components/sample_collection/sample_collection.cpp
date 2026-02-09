#include "sample_collection.hpp"
#include <imu_sampler.hpp>
#include <fs.hpp>
#include <rtc.hpp>
#include <terminal.hpp>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <cstring>
#include <cstdio>

static const char* TAG = "SAMPLE_COLLECTION";

// Minimum free space to continue collection (16KB - enough for final writes + FS overhead)
static constexpr size_t MIN_FREE_SPACE = 16 * 1024;

// Global state
static struct {
    bool initialized;
    bool active;
    QueueHandle_t queue;
    TaskHandle_t task_handle;
    FILE* file;
    sample_collection_stats_t stats;
} g_collection = {};

// Forward declaration
static void sample_collection_task(void* pvParameters);

bool sample_collection_init(void) {
    if (g_collection.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return true;
    }

    // Get queue from IMU sampler
    g_collection.queue = (QueueHandle_t)imu_sampler_get_queue();
    if (!g_collection.queue) {
        ESP_LOGE(TAG, "Failed to get IMU sampler queue");
        return false;
    }

    // Create collection task (normal priority)
    BaseType_t ret = xTaskCreatePinnedToCore(
        sample_collection_task,
        "sample_collect",
        8192,  // Stack size - larger for file operations
        NULL,
        5,     // Normal priority
        &g_collection.task_handle,
        0      // Core 0
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        return false;
    }

    g_collection.initialized = true;
    ESP_LOGI(TAG, "Initialized successfully");

    return true;
}

bool sample_collection_start(void) {
    if (!g_collection.initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return false;
    }

    if (g_collection.active) {
        ESP_LOGW(TAG, "Already active");
        return true;
    }

    // Check file system
    if (!fs_is_mounted()) {
        ESP_LOGE(TAG, "File system not mounted");
        terminal_write("FS not mounted!");
        return false;
    }

    // Check available space
    size_t total, used;
    if (fs_get_info(&total, &used)) {
        size_t free = total - used;
        if (free < MIN_FREE_SPACE) {
            ESP_LOGE(TAG, "Insufficient space: %d bytes free", free);
            terminal_write("Insufficient space!");
            return false;
        }
        ESP_LOGI(TAG, "Available space: %d KB", free / 1024);
    }

    // Reset statistics
    memset(&g_collection.stats, 0, sizeof(g_collection.stats));

    // Create filename with timestamp
    char time_str[32];
    rtc_get_time_string(time_str, sizeof(time_str));

    // Create safe filename (replace spaces, colons, dashes with underscores)
    char safe_time[32];
    int j = 0;
    for (int i = 0; time_str[i] != '\0' && j < 31; i++) {
        if (time_str[i] == ' ' || time_str[i] == ':' || time_str[i] == '-') {
            safe_time[j++] = '_';
        } else {
            safe_time[j++] = time_str[i];
        }
    }
    safe_time[j] = '\0';

    snprintf(g_collection.stats.current_filename, 
             sizeof(g_collection.stats.current_filename),
             "/spiffs/imu_%s.bin", safe_time);

    // Open file for writing
    g_collection.file = fopen(g_collection.stats.current_filename, "wb");
    if (!g_collection.file) {
        ESP_LOGE(TAG, "Failed to create file: %s", g_collection.stats.current_filename);
        terminal_write("Failed to create file");
        return false;
    }

    ESP_LOGI(TAG, "Created file: %s", g_collection.stats.current_filename);
    terminal_write("File: %s", g_collection.stats.current_filename);

    // Write file header
    struct file_header_t {
        uint32_t magic;          // 'IMU1'
        uint32_t version;        // 1
        uint64_t start_time_us;  // Start timestamp
        uint32_t sample_rate;    // 1000 Hz
        uint16_t accel_fsr;      // 8G
        uint16_t gyro_fsr;       // 2000 DPS
    } __attribute__((packed));

    file_header_t header = {
        .magic = 0x31554D49,  // 'IMU1' in little endian
        .version = 1,
        .start_time_us = static_cast<uint64_t>(esp_timer_get_time()),
        .sample_rate = 1000,
        .accel_fsr = 8,
        .gyro_fsr = 2000
    };

    size_t written = fwrite(&header, 1, sizeof(header), g_collection.file);
    if (written != sizeof(header)) {
        ESP_LOGE(TAG, "Failed to write header");
        fclose(g_collection.file);
        g_collection.file = nullptr;
        return false;
    }

    fflush(g_collection.file);
    g_collection.stats.total_bytes_written = sizeof(header);

    // Start IMU sampling
    if (!imu_sampler_start()) {
        ESP_LOGE(TAG, "Failed to start IMU sampler");
        fclose(g_collection.file);
        g_collection.file = nullptr;
        terminal_write("Failed to start IMU");
        return false;
    }

    g_collection.active = true;
    ESP_LOGI(TAG, "Collection started");
    terminal_write("Collection STARTED");

    return true;
}

bool sample_collection_stop(void) {
    if (!g_collection.active) {
        return true;
    }

    // Stop IMU sampling first
    imu_sampler_stop();

    g_collection.active = false;

    // Give task time to process remaining samples
    vTaskDelay(pdMS_TO_TICKS(100));

    // Close file if open
    if (g_collection.file) {
        fflush(g_collection.file);
        fclose(g_collection.file);
        g_collection.file = nullptr;
        ESP_LOGI(TAG, "File closed: %s", g_collection.stats.current_filename);
    }

    // Get final IMU stats
    imu_sampler_stats_t imu_stats;
    imu_sampler_get_stats(&imu_stats);

    ESP_LOGI(TAG, "Collection stopped");
    terminal_write("Collection STOPPED");
    terminal_write("Samples: %llu | Batches: %u | Bytes: %llu",
                   g_collection.stats.total_samples_written,
                   g_collection.stats.batches_written,
                   g_collection.stats.total_bytes_written);
    terminal_write("IMU: %llu samples | %u overflows",
                   imu_stats.total_samples,
                   imu_stats.fifo_overflows);

    return true;
}

bool sample_collection_is_active(void) {
    return g_collection.active;
}

void sample_collection_get_stats(sample_collection_stats_t* stats) {
    if (stats) {
        *stats = g_collection.stats;
    }
}

// Collection task - processes batches from queue and writes to file
static void sample_collection_task(void* pvParameters) {
    (void)pvParameters;

    imu_batch_t batch;

    ESP_LOGI(TAG, "Collection task started");

    while (true) {
        // Wait for batch from queue (blocking, 1 second timeout)
        if (xQueueReceive(g_collection.queue, &batch, pdMS_TO_TICKS(1000)) == pdTRUE) {
            
            if (!g_collection.active || !g_collection.file) {
                // Not collecting, discard batch
                continue;
            }

            // Check available space every 10 batches
            if (g_collection.stats.batches_written % 10 == 0) {
                size_t total, used;
                if (fs_get_info(&total, &used)) {
                    size_t free = total - used;
                    if (free < MIN_FREE_SPACE) {
                        ESP_LOGW(TAG, "Out of space: %d bytes free", free);
                        g_collection.stats.space_full_stops++;
                        
                        // Stop collection
                        terminal_write("Storage full - stopping");
                        sample_collection_stop();
                        continue;
                    }
                }
            }

            // Write batch to file
            // Format: [timestamp_us(8) | sample_count(2) | samples(14 * count)]
            struct batch_file_header_t {
                uint64_t base_timestamp_us;
                uint16_t sample_count;
            } __attribute__((packed));

            batch_file_header_t batch_header = {
                .base_timestamp_us = batch.base_timestamp_us,
                .sample_count = batch.sample_count
            };

            // Write batch header
            size_t written = fwrite(&batch_header, 1, sizeof(batch_header), g_collection.file);
            if (written != sizeof(batch_header)) {
                ESP_LOGE(TAG, "Failed to write batch header");
                g_collection.stats.write_errors++;
                continue;
            }

            // Write samples
            size_t sample_bytes = batch.sample_count * sizeof(imu_sample_t);
            written = fwrite(batch.samples, 1, sample_bytes, g_collection.file);
            if (written != sample_bytes) {
                ESP_LOGE(TAG, "Failed to write samples");
                g_collection.stats.write_errors++;
                continue;
            }

            // Flush every 10 batches to ensure data is written
            if (g_collection.stats.batches_written % 10 == 0) {
                fflush(g_collection.file);
            }

            // Update statistics
            g_collection.stats.total_samples_written += batch.sample_count;
            g_collection.stats.total_bytes_written += sizeof(batch_header) + sample_bytes;
            g_collection.stats.batches_written++;
        }
    }
}
