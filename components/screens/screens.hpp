#pragma once
#include <M5Unified.hpp>
#include <terminal.hpp>

/**
 * Initialize the screens system
 * Must be called after terminal_init()
 * @param display Pointer to the M5GFX display object
 */
void screens_init(M5GFX* display);

/**
 * Update the current screen
 * Call this regularly in main loop
 * Handles screen switching and updates based on system state
 */
void screens_update();

/**
 * Draw the recording screen
 * @param display Pointer to the M5GFX display object
 */
void draw_recording_screen(M5GFX* display);
