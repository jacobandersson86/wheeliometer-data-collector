#include "buttons.hpp"
#include "terminal.hpp"
#include "fs.hpp"
#include "rtc.hpp"
#include "wifi.hpp"
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
        terminal_write("Button A clicked");

        // Get current time for filename
        char filename[64];
        char time_str[32];
        rtc_get_time_string(time_str, sizeof(time_str));

        // Create filename from timestamp (replace spaces and colons with underscores)
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

        snprintf(filename, sizeof(filename), "/data_%s.txt", safe_time);

        // Generate some jibberish data
        char data[256];
        int len = 0;
        len += snprintf(data + len, sizeof(data) - len, "File created at: %s\n", time_str);
        len += snprintf(data + len, sizeof(data) - len, "Random jibberish data:\n");

        // Add some random-looking data
        for (int i = 0; i < 10; i++) {
            len += snprintf(data + len, sizeof(data) - len,
                          "Line %d: %d %d %d %d\n",
                          i, rand() % 1000, rand() % 1000, rand() % 1000, rand() % 1000);
        }

        // Write to file
        int written = fs_write_file(filename, data, len);
        if (written > 0) {
            terminal_write("Created file: %s (%d bytes)", filename, written);
        } else {
            terminal_write("Failed to create file!");
        }
    }

    if (M5.BtnA.wasHold()) {
        terminal_write("Button A held");
    }

    if (M5.BtnB.wasClicked()) {
        terminal_write("Button B clicked - Toggling WiFi AP");
        
        if (wifi_is_ap_running()) {
            wifi_stop_ap();
        } else {
            wifi_start_ap();
        }
    }

    if (M5.BtnPWR.wasClicked()) {
        terminal_write("Power button clicked");
    }
}
