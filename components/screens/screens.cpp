#include "screens.hpp"
#include <terminal.hpp>
#include <sample_collection.hpp>

static struct {
    M5GFX* display;
    screen_type_t current_screen;
    bool initialized;
} screens_state;

void screens_init(M5GFX* display) {
    screens_state.display = display;
    screens_state.current_screen = SCREEN_TERMINAL;
    screens_state.initialized = true;
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
            terminal_deactivate();
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
        }
        // Recording screen is static, no updates needed
    }
}

void draw_recording_screen(M5GFX* display) {
    display->fillScreen(TFT_BLACK);

    // Calculate center position
    const char* text = "Recording";
    int16_t text_width = display->textWidth(text);
    int16_t text_height = display->fontHeight();
    int16_t x = (display->width() - text_width) / 2;
    int16_t y = (display->height() - text_height) / 2;

    // Draw text in center
    display->setTextColor(TFT_RED);
    display->setCursor(x, y);
    display->print(text);
    display->setTextColor(TFT_WHITE);  // Reset to white
}
