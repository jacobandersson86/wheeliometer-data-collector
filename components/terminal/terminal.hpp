#pragma once
#include <M5Unified.hpp>


void terminal_init(M5GFX* display);
void terminal_write(const char * format, ...);
int16_t terminal_get_max_chars_per_line();
