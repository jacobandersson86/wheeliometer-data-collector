#pragma once
#include <M5Unified.hpp>

// Screen types
enum screen_type_t {
    SCREEN_TERMINAL,
    SCREEN_RECORDING
};

void terminal_init(M5GFX* display);
void terminal_write(const char * format, ...);
int16_t terminal_get_max_chars_per_line();

// Functions for screen management
void terminal_redraw();     // Called when switching to terminal screen - redraws buffered content
void terminal_update();     // Called for regular updates when on terminal screen
void terminal_deactivate(); // Called when switching away from terminal screen
