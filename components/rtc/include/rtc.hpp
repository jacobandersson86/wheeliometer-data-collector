#pragma once
#include <stddef.h>

// RTC initialization mode
typedef enum {
    RTC_MODE_COMPILE_TIME = 0,  // Set to compile time on first boot
    RTC_MODE_SERIAL = 1          // Wait for serial command to set time
} rtc_init_mode_t;

// Initialize the RTC module
// mode: RTC_MODE_COMPILE_TIME or RTC_MODE_SERIAL
void rtc_init(rtc_init_mode_t mode);

// Check for serial commands and update RTC if command received
// Should be called regularly in main loop
void rtc_check_serial();

// Check if RTC has valid time (year between 2020-2100)
bool rtc_has_valid_time();

// Set RTC to compile time
void rtc_set_compile_time();

// Get formatted time string
// buffer: output buffer for the time string
// buffer_size: size of the output buffer
// Returns: pointer to buffer
char* rtc_get_time_string(char* buffer, size_t buffer_size);
