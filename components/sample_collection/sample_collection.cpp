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

// Minimum free space to continue collection (64KB - SPIFFS needs more headroom due to fragmentation)
static constexpr size_t MIN_FREE_SPACE = 64 * 1024;

// Maximum consecutive write failures before stopping
static constexpr uint32_t MAX_CONSECUTIVE_FAILURES = 5;

// Stop if error rate exceeds this percentage over recent batches
static constexpr uint32_t ERROR_RATE_WINDOW = 50;  // Check last 50 batches
static constexpr uint32_t MAX_ERROR_RATE_PERCENT = 20;  // Stop if >20% errors

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

    // Create collection task (higher priority than sampler to prevent queue overflow)
    BaseType_t ret = xTaskCreatePinnedToCore(
        sample_collection_task,
        "sample_collect",
        8192,  // Stack size - larger for file operations
        NULL,
        21,    // Higher than sampler (20) to drain queue faster
        &g_collection.task_handle,
        1      // Core 1 - same as sampler for better cache coherency
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

    // Set large write buffer (8KB) to reduce SPIFFS operations
    static char write_buffer[8192];
    setvbuf(g_collection.file, write_buffer, _IOFBF, sizeof(write_buffer));

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
    uint32_t last_debug_time = 0;
    uint32_t batches_since_debug = 0;
    int64_t write_start_us = 0;
    int64_t total_write_time_us = 0;
    uint32_t consecutive_failures = 0;
    uint32_t recent_batches = 0;
    uint32_t recent_errors = 0;

    ESP_LOGI(TAG, "Collection task started (priority %d, core %d)",
             uxTaskPriorityGet(NULL), xPortGetCoreID());

    while (true) {
        // Wait for batch from queue (blocking, 1 second timeout)
        if (xQueueReceive(g_collection.queue, &batch, pdMS_TO_TICKS(1000)) == pdTRUE) {

            if (!g_collection.active || !g_collection.file) {
                // Not collecting, discard batch
                continue;
            }

            // Debug: Log queue usage every 5 seconds
            uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now_ms - last_debug_time >= 5000) {
                UBaseType_t queue_items = uxQueueMessagesWaiting(g_collection.queue);
                uint32_t avg_write_us = batches_since_debug > 0 ?
                    (total_write_time_us / batches_since_debug) : 0;
                ESP_LOGI(TAG, "Queue: %d/50 items | Avg write: %lu us/batch | Batches: %u",
                         queue_items, avg_write_us, g_collection.stats.batches_written);
                last_debug_time = now_ms;
                batches_since_debug = 0;
                total_write_time_us = 0;
            }

            write_start_us = esp_timer_get_time();

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
                g_collection.stats.write_errors++;
                consecutive_failures++;
                recent_errors++;

                // Track error rate in sliding window
                if (recent_batches >= ERROR_RATE_WINDOW) {
                    uint32_t error_rate = (recent_errors * 100) / recent_batches;
                    if (error_rate > MAX_ERROR_RATE_PERCENT) {
                        ESP_LOGW(TAG, "High error rate: %u%% over %u batches", error_rate, recent_batches);
                        terminal_write("Too many write errors - stopping");
                        g_collection.stats.space_full_stops++;
                        sample_collection_stop();
                        continue;
                    }
                }

                // Check space and log
                size_t total, used;
                if (fs_get_info(&total, &used)) {
                    size_t free = total - used;
                    ESP_LOGE(TAG, "Failed to write batch header (free: %d KB, consecutive: %u, recent errors: %u/%u)",
                             free / 1024, consecutive_failures, recent_errors, recent_batches);

                    // Stop if out of space OR too many consecutive failures
                    if (free < MIN_FREE_SPACE || consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
                        ESP_LOGW(TAG, "Stopping collection: %s",
                                 free < MIN_FREE_SPACE ? "out of space" : "too many consecutive failures");
                        g_collection.stats.space_full_stops++;
                        terminal_write("Storage full - stopping");
                        sample_collection_stop();
                    }
                } else {
                    ESP_LOGE(TAG, "Failed to write batch header (consecutive: %u)", consecutive_failures);
                }
                continue;
            }

            // Write samples
            size_t sample_bytes = batch.sample_count * sizeof(imu_sample_t);
            written = fwrite(batch.samples, 1, sample_bytes, g_collection.file);
            if (written != sample_bytes) {
                g_collection.stats.write_errors++;
                consecutive_failures++;
                recent_errors++;

                // Track error rate in sliding window
                if (recent_batches >= ERROR_RATE_WINDOW) {
                    uint32_t error_rate = (recent_errors * 100) / recent_batches;
                    if (error_rate > MAX_ERROR_RATE_PERCENT) {
                        ESP_LOGW(TAG, "High error rate: %u%% over %u batches", error_rate, recent_batches);
                        terminal_write("Too many write errors - stopping");
                        g_collection.stats.space_full_stops++;
                        sample_collection_stop();
                        continue;
                    }
                }

                // Check space and log
                size_t total, used;
                if (fs_get_info(&total, &used)) {
                    size_t free = total - used;
                    ESP_LOGE(TAG, "Failed to write samples (free: %d KB, consecutive: %u, recent errors: %u/%u)",
                             free / 1024, consecutive_failures, recent_errors, recent_batches);

                    // Stop if out of space OR too many consecutive failures
                    if (free < MIN_FREE_SPACE || consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
                        ESP_LOGW(TAG, "Stopping collection: %s",
                                 free < MIN_FREE_SPACE ? "out of space" : "too many consecutive failures");
                        g_collection.stats.space_full_stops++;
                        terminal_write("Storage full - stopping");
                        sample_collection_stop();
                    }
                } else {
                    ESP_LOGE(TAG, "Failed to write samples (consecutive: %u)", consecutive_failures);
                }
                continue;
            }

            // Reset failure counter and track successful batch
            consecutive_failures = 0;
            recent_batches++;

            // Reset sliding window after full cycle
            if (recent_batches > ERROR_RATE_WINDOW * 2) {
                recent_batches = ERROR_RATE_WINDOW;
                recent_errors = (recent_errors * ERROR_RATE_WINDOW) / (ERROR_RATE_WINDOW * 2);
            }

            // Update statistics
            g_collection.stats.total_samples_written += batch.sample_count;
            g_collection.stats.total_bytes_written += sizeof(batch_header) + sample_bytes;
            g_collection.stats.batches_written++;

            // Track write performance
            int64_t write_duration_us = esp_timer_get_time() - write_start_us;
            total_write_time_us += write_duration_us;
            batches_since_debug++;

            // Warn on slow writes (>500ms is unusual, 200-300ms is normal for SPIFFS wear leveling)
            if (write_duration_us > 500000) {  // >500ms
                ESP_LOGW(TAG, "Slow write: %lld ms for batch %u",
                         write_duration_us / 1000, g_collection.stats.batches_written);
            }
        }
    }
}
