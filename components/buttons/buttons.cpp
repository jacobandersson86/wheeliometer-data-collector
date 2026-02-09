#include "buttons.hpp"
#include "terminal.hpp"
#include "fs.hpp"
#include "rtc.hpp"
#include "wifi.hpp"
#include "webserver.hpp"
#include "imu_sampler.hpp"
#include "imu_consumer.hpp"
#include <M5Unified.hpp>
#include <stdio.h>
#include <stdlib.h>

void buttons_init() {
    // Button configuration can go here if needed
    // For example: M5.BtnA.setDebounceThresh(20);
}

void buttons_check() {
    // Check button states immediately after M5.update()
    // was* methods are edge-triggered and only return true once per event

    if (M5.BtnA.wasClicked()) {
        // Toggle IMU sampling
        if (imu_sampler_is_running()) {
            imu_sampler_stop();
            terminal_write("IMU sampling STOPPED");

            // Print final statistics
            imu_sampler_stats_t stats;
            imu_sampler_get_stats(&stats);
            terminal_write("Stats: %llu samples, %u batches, %u overflows, %u drops",
                         stats.total_samples, stats.batches_sent,
                         stats.fifo_overflows, stats.queue_full_errors);
        } else {
            // Reset averages when starting
            imu_consumer_reset_averages();
            imu_sampler_reset_stats();

            if (imu_sampler_start()) {
                terminal_write("IMU sampling STARTED (1kHz)");
            } else {
                terminal_write("Failed to start IMU sampling");
            }
        }
    }

    if (M5.BtnA.wasHold()) {
        terminal_write("Button A held");
    }

    if (M5.BtnB.wasClicked()) {
        terminal_write("Button B clicked - Toggling WiFi AP");

        if (wifi_is_ap_running()) {
            // Stop webserver first, then WiFi
            webserver_stop();
            wifi_stop_ap();
        } else {
            // Start WiFi first, then webserver
            wifi_start_ap();
            webserver_start();
        }
    }

    if (M5.BtnPWR.wasClicked()) {
        terminal_write("Power button clicked");
    }
}
