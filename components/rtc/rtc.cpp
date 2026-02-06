#include "rtc.hpp"
#include "terminal.hpp"
#include <M5Unified.hpp>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include "esp_timer.h"
#include "esp_log.h"

static const char* TAG = "RTC";

static rtc_init_mode_t rtc_mode = RTC_MODE_COMPILE_TIME;
static char serial_buffer[128];
static int buffer_pos = 0;
static bool stdin_configured = false;

bool rtc_has_valid_time() {
    auto dt = M5.Rtc.getDateTime();
    // Check if year is reasonable (after 2020 and before 2100)
    return (dt.date.year >= 2020 && dt.date.year < 2100);
}

void rtc_set_compile_time() {
    // __DATE__ format: "Jan 23 2026"
    // __TIME__ format: "12:34:56"

    const char* months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                           "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

    char month_str[4];
    int day, year, hour, minute, second;

    sscanf(__DATE__, "%s %d %d", month_str, &day, &year);
    sscanf(__TIME__, "%d:%d:%d", &hour, &minute, &second);

    int month = 1;
    for (int i = 0; i < 12; i++) {
        if (strcmp(month_str, months[i]) == 0) {
            month = i + 1;
            break;
        }
    }

    // Calculate day of week (Zeller's congruence for Gregorian calendar)
    int y = year;
    int m = month;
    if (m < 3) {
        m += 12;
        y -= 1;
    }
    int weekday = (day + (13 * (m + 1)) / 5 + y + y / 4 - y / 100 + y / 400) % 7;
    // Adjust: Zeller gives 0=Saturday, we want 0=Sunday
    weekday = (weekday + 6) % 7;

    m5::rtc_datetime_t datetime;
    datetime.date.year = year;
    datetime.date.month = month;
    datetime.date.date = day;
    datetime.date.weekDay = weekday;
    datetime.time.hours = hour;
    datetime.time.minutes = minute;
    datetime.time.seconds = second;

    M5.Rtc.setDateTime(datetime);

    char msg[100];
    snprintf(msg, sizeof(msg), "RTC set to compile time: %04d-%02d-%02d %02d:%02d:%02d",
             year, month, day, hour, minute, second);
    terminal_write(msg);
}

void rtc_check_serial() {
    // Only check serial if in serial mode
    if (rtc_mode != RTC_MODE_SERIAL) {
        return;
    }

    // Throttle: only check every 100ms to avoid watchdog issues
    static uint32_t last_check = 0;
    uint32_t now = esp_timer_get_time() / 1000; // Convert to milliseconds
    if (now - last_check < 100) {
        return;
    }
    last_check = now;

    // Configure stdin for non-blocking mode once
    if (!stdin_configured) {
        fcntl(fileno(stdin), F_SETFL, O_NONBLOCK);
        stdin_configured = true;
    }

    // Read available characters from stdin (non-blocking)
    int c;
    while ((c = fgetc(stdin)) != EOF) {
        if (c == '\n' || c == '\r') {
            if (buffer_pos > 0) {
                serial_buffer[buffer_pos] = '\0';

                // Check for SETRTC command: SETRTC:YYYY,MM,DD,HH,MM,SS,WEEKDAY
                if (strncmp(serial_buffer, "SETRTC:", 7) == 0) {
                    int year, month, day, hour, minute, second, weekday;
                    int parsed = sscanf(serial_buffer + 7, "%d,%d,%d,%d,%d,%d,%d",
                              &year, &month, &day, &hour, &minute, &second, &weekday);

                    if (parsed == 7) {
                        m5::rtc_datetime_t datetime;
                        datetime.date.year = year;
                        datetime.date.month = month;
                        datetime.date.date = day;
                        datetime.date.weekDay = weekday;
                        datetime.time.hours = hour;
                        datetime.time.minutes = minute;
                        datetime.time.seconds = second;

                        M5.Rtc.setDateTime(datetime);

                        char msg[100];
                        snprintf(msg, sizeof(msg), "RTC set via serial: %04d-%02d-%02d %02d:%02d:%02d",
                                year, month, day, hour, minute, second);
                        terminal_write(msg);
                        printf("OK: RTC time set\n");
                        ESP_LOGI(TAG, "RTC time set successfully");
                    } else {
                        printf("ERROR: Invalid SETRTC format (parsed %d/7 values)\n", parsed);
                        ESP_LOGE(TAG, "Invalid SETRTC format: parsed %d/7 values from '%s'", parsed, serial_buffer);
                    }
                }

                buffer_pos = 0;
            }
        } else if (buffer_pos < (int)sizeof(serial_buffer) - 1) {
            serial_buffer[buffer_pos++] = c;
        }
    }
}

void rtc_init(rtc_init_mode_t mode) {
    rtc_mode = mode;

    if (!M5.Rtc.isEnabled()) {
        terminal_write("RTC not detected!");
        return;
    }

    terminal_write("RTC detected");

    // Check if RTC already has valid time
    if (!rtc_has_valid_time()) {
        if (mode == RTC_MODE_COMPILE_TIME) {
            terminal_write("RTC time invalid, setting to compile time...");
            rtc_set_compile_time();
        } else {
            terminal_write("RTC time invalid. Send SETRTC command via serial.");
            terminal_write("Or run: python set_rtc_time.py");
        }
    } else {
        auto dt = M5.Rtc.getDateTime();
        char msg[100];
        snprintf(msg, sizeof(msg), "RTC time: %04d-%02d-%02d %02d:%02d:%02d",
                dt.date.year, dt.date.month, dt.date.date,
                dt.time.hours, dt.time.minutes, dt.time.seconds);
        terminal_write(msg);
    }
}

char* rtc_get_time_string(char* buffer, size_t buffer_size) {
    if (!M5.Rtc.isEnabled() || !buffer || buffer_size == 0) {
        if (buffer && buffer_size > 0) {
            buffer[0] = '\0';
        }
        return buffer;
    }

    auto dt = M5.Rtc.getDateTime();
    snprintf(buffer, buffer_size, "%04d-%02d-%02d %02d:%02d:%02d",
            dt.date.year, dt.date.month, dt.date.date,
            dt.time.hours, dt.time.minutes, dt.time.seconds);

    return buffer;
}
