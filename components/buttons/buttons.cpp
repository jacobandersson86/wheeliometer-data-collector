#include "buttons.hpp"
#include "terminal.hpp"
#include "fs.hpp"
#include "rtc.hpp"
#include "wifi.hpp"
#include "webserver.hpp"
#include "sample_collection.hpp"
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
        // Toggle sample collection
        if (sample_collection_is_active()) {
            sample_collection_stop();
        } else {
            sample_collection_start();
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
