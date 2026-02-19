#include "screens.hpp"
#include <terminal.hpp>
#include <sample_collection.hpp>
#include <fs.hpp>
#include "esp_timer.h"
#include <cstring>

static struct {
    M5GFX* display;
    screen_type_t current_screen;
    bool initialized;
    int64_t last_update_time_us;  // Last time recording screen was updated
    size_t initial_free_space;     // Free space when recording started
} screens_state;

void screens_init(M5GFX* display) {
    screens_state.display = display;
    screens_state.current_screen = SCREEN_TERMINAL;
    screens_state.initialized = true;
    screens_state.last_update_time_us = 0;
    screens_state.initial_free_space = 0;
}

void screens_update() {
    if (!screens_state.initialized) {
        return;
    }

    // Determine what screen should be displayed based on system state
    screen_type_t desired_screen = sample_collection_is_active() ? SCREEN_RECORDING : SCREEN_TERMINAL;

    // Check if we need to switch screens
    if (desired_screen != screens_state.current_screen) {
        screens_state.current_screen = desired_screen;

        if (desired_screen == SCREEN_TERMINAL) {
            // Switching to terminal - redraw all buffered content
            terminal_redraw();
        } else {
            // Switching away from terminal - deactivate it first

            // Capture initial free space when starting recording
            size_t total_bytes = 0, used_bytes = 0;
            if (fs_get_info(&total_bytes, &used_bytes)) {
                screens_state.initial_free_space = total_bytes - used_bytes;
            }

            // Then draw the recording screen
            // We need to get the display from somewhere - let's store it
            if (screens_state.display) {
                draw_recording_screen(screens_state.display);
            }
        }
    } else {
        // No screen switch - do regular updates
        if (screens_state.current_screen == SCREEN_TERMINAL) {
            terminal_update();
        } else if (screens_state.current_screen == SCREEN_RECORDING) {
            // Update recording screen periodically (every 1 second)
            int64_t now_us = esp_timer_get_time();
            if (now_us - screens_state.last_update_time_us >= 1000000) {  // 1 second
                screens_state.last_update_time_us = now_us;
                if (screens_state.display) {
                    draw_recording_screen(screens_state.display);
                }
            }
        }
    }
}

void draw_recording_screen(M5GFX* display) {
    display->fillScreen(TFT_BLACK);

    // Get current file size from sample collection stats
    sample_collection_stats_t stats;
    sample_collection_get_stats(&stats);

    // Extract just the filename (without path)
    const char* filename = stats.current_filename;
    const char* last_slash = strrchr(filename, '/');
    if (last_slash) {
        filename = last_slash + 1;
    }

    // Draw "Recording" text in upper portion
    const char* title = "Recording";
    int16_t title_width = display->textWidth(title);
    int16_t title_height = display->fontHeight();
    int16_t title_x = (display->width() - title_width) / 2;
    int16_t title_y = display->height() / 3;  // Upper third of screen

    display->setTextColor(TFT_RED);
    display->setCursor(title_x, title_y);
    display->print(title);

    // Draw filename below title
    int16_t filename_width = display->textWidth(filename);
    int16_t filename_x = (display->width() - filename_width) / 2;
    int16_t filename_y = title_y + title_height + 8;

    display->setTextColor(TFT_WHITE);
    display->setCursor(filename_x, filename_y);
    display->print(filename);

    // Draw memory usage info below filename
    char mem_info[64];
    // Show current file size and initial free space in KB
    snprintf(mem_info, sizeof(mem_info), "%llu KB / %u KB",
             stats.total_bytes_written / 1024,
             (unsigned int)(screens_state.initial_free_space / 1024));

    int16_t info_width = display->textWidth(mem_info);
    int16_t info_x = (display->width() - info_width) / 2;
    int16_t info_y = filename_y + title_height + 8;  // Below filename with spacing

    display->setCursor(info_x, info_y);
    display->print(mem_info);
}
