#pragma once
#include <stdbool.h>

// Initialize WiFi subsystem
// Must be called before any other wifi_ functions
bool wifi_init();

// Start WiFi Access Point
// SSID: "Wheeliometer" (open network, no password)
// Returns true on success
bool wifi_start_ap();

// Stop WiFi Access Point
// Returns true on success
bool wifi_stop_ap();

// Check if AP is currently running
bool wifi_is_ap_running();

// Get number of connected stations
int wifi_get_station_count();
