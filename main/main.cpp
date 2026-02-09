#include <M5Unified.h>
#include <stdio.h>
#include <terminal.hpp>
#include <buttons.hpp>
#include <rtc.hpp>
#include <fs.hpp>
#include <wifi.hpp>
#include <webserver.hpp>
#include <imu_sampler.hpp>
#include <sample_collection.hpp>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

// Main application task
void main_task(void *pvParameters) {
    // Subscribe this task to the watchdog
    esp_task_wdt_add(NULL);

    while (true) {
        M5.delay(10);  // Delay 10ms to give IDLE task more time
        M5.update();
        buttons_check();
        rtc_check_serial();  // Check for serial commands

        // Reset the watchdog for this task
        esp_task_wdt_reset();
    }
}

extern "C" {
    void app_main(void)
    {
        auto cfg = M5.config();
        M5.begin(cfg);

        buttons_init();

        terminal_init(&M5.Display);
        terminal_write("Wheeliometer startup complete");

        // Initialize RTC
        // Use RTC_MODE_COMPILE_TIME for automatic compile-time setting
        // Use RTC_MODE_SERIAL to set time via serial with set_rtc_time.py
        rtc_init(RTC_MODE_SERIAL);

        // Initialize file system
        if (fs_init()) {
            terminal_write("File system ready");

            // Display file system info
            size_t total, used;
            if (fs_get_info(&total, &used)) {
                terminal_write("FS: %d/%d bytes (%.1f%% used)",
                              used, total, (100.0 * used) / total);
            }
        } else {
            terminal_write("File system init failed!");
        }

        // Initialize WiFi subsystem (but don't start AP yet)
        wifi_init();

        // Initialize webserver subsystem (but don't start server yet)
        webserver_init();

        // Initialize IMU sampler
        imu_sampler_config_t imu_config = {
            .accel_fsr = 2,     // AFS_8G (0=2G, 1=4G, 2=8G, 3=16G)
            .gyro_fsr = 3,      // GFS_2000DPS (0=250, 1=500, 2=1000, 3=2000)
            .odr = 0,           // ODR_1kHz (0=1kHz, 1=500Hz, 3=250Hz...)
            .int_pin = 35,      // INT pin for M5StickC Plus (GPIO 35)
            .queue_depth = 10   // Queue can hold 10 batches
        };

        if (imu_sampler_init(&imu_config)) {
            terminal_write("IMU sampler initialized");

            // Initialize sample collection
            if (sample_collection_init()) {
                terminal_write("Sample collection ready");
                terminal_write("Press Button A to start/stop");
            } else {
                terminal_write("Sample collection init failed!");
            }
        } else {
            terminal_write("IMU sampler init failed!");
        }

        // Create main application task
        xTaskCreate(main_task, "main_task", 4096, NULL, 5, NULL);

        // app_main should return, not loop
    }
}

