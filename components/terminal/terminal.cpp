#include "terminal.hpp"
#include <M5Unified.hpp>
#include <cstdarg>
#include <cstring>

// Terminal configuration constants
static const int16_t TERMINAL_MARGIN_HORIZONTAL = 10;  // Left and right margins (pixels)
static const int16_t TERMINAL_MARGIN_VERTICAL = 5;     // Top and bottom margins (pixels)
static const int16_t TERMINAL_LINE_SPACING = 2;        // Spacing between lines (pixels)
static const uint8_t TERMINAL_ROTATION = 3;            // Display rotation
static const float TERMINAL_TEXT_SIZE = 1.2;           // Text size multiplier
static const size_t TERMINAL_BUFFER_SIZE = 256;        // Buffer size for formatted strings
static const size_t TERMINAL_MAX_LINE_LENGTH = 128;    // Max length of a single line in buffer
static const size_t TERMINAL_LINE_BUFFER_SIZE = 50;    // Number of lines to buffer

static struct {
    M5GFX* display;
    int16_t margin_horizontal;
    int16_t margin_vertical;
    int16_t x_offset;
    int16_t y_offset;
    int16_t current_y;
    int16_t line_height;
    int16_t line_spacing;
    int16_t max_y;
    int16_t content_width;
    bool is_active;  // Whether terminal should draw to display

    // Line buffer for redrawing terminal
    char line_buffer[TERMINAL_LINE_BUFFER_SIZE][TERMINAL_MAX_LINE_LENGTH];
    size_t line_buffer_count;  // Number of lines stored
    size_t line_buffer_start;  // Index of oldest line (circular buffer)
} terminal;


// Helper function to add a line to the circular buffer
static void add_line_to_buffer(const char* line, size_t len) {
    if (len >= TERMINAL_MAX_LINE_LENGTH) {
        len = TERMINAL_MAX_LINE_LENGTH - 1;
    }

    // Calculate the index to write to
    size_t write_index;
    if (terminal.line_buffer_count < TERMINAL_LINE_BUFFER_SIZE) {
        // Buffer not full yet, just append
        write_index = terminal.line_buffer_count;
        terminal.line_buffer_count++;
    } else {
        // Buffer full, overwrite oldest line
        write_index = terminal.line_buffer_start;
        terminal.line_buffer_start = (terminal.line_buffer_start + 1) % TERMINAL_LINE_BUFFER_SIZE;
    }

    // Copy line to buffer
    memcpy(terminal.line_buffer[write_index], line, len);
    terminal.line_buffer[write_index][len] = '\0';
}

// Helper function to draw a single line on display
static void draw_line_on_display(const char* line, int16_t y_pos) {
    terminal.display->setCursor(terminal.x_offset, y_pos);
    terminal.display->print(line);
}

void terminal_init(M5GFX* display) {
    terminal.display = display;
    terminal.display->setRotation(TERMINAL_ROTATION);
    terminal.display->setTextSize(TERMINAL_TEXT_SIZE);

    // Initialize terminal parameters with margins
    terminal.margin_horizontal = TERMINAL_MARGIN_HORIZONTAL;
    terminal.margin_vertical = TERMINAL_MARGIN_VERTICAL;
    terminal.x_offset = terminal.margin_horizontal;
    terminal.y_offset = terminal.margin_vertical;
    terminal.line_height = terminal.display->fontHeight();
    terminal.line_spacing = TERMINAL_LINE_SPACING;
    terminal.max_y = terminal.display->height() - terminal.margin_vertical;
    terminal.content_width = terminal.display->width() - (2 * terminal.margin_horizontal);
    terminal.current_y = terminal.y_offset;
    terminal.is_active = true;  // Terminal is active by default

    // Initialize line buffer
    terminal.line_buffer_count = 0;
    terminal.line_buffer_start = 0;
    memset(terminal.line_buffer, 0, sizeof(terminal.line_buffer));

    // Clear display and set initial cursor
    terminal.display->fillScreen(TFT_BLACK);
    terminal.display->setCursor(terminal.x_offset, terminal.current_y);
}

void terminal_write(const char *format, ...) {
    // Format the string using variadic arguments
    char buffer[TERMINAL_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    int16_t text_len = strlen(buffer);

    // Check if string ends with newline
    bool ends_with_newline = false;
    if (text_len > 0 && buffer[text_len - 1] == '\n') {
        ends_with_newline = true;
        text_len--;  // Don't include the newline in text processing
    }

    int16_t max_chars = terminal_get_max_chars_per_line();
    int16_t pos = 0;

    // If text is empty, just advance one line
    if (text_len == 0 && !ends_with_newline) {
        // Add empty line to buffer
        add_line_to_buffer("", 0);

        // Only update display if terminal is active
        if (terminal.is_active) {
            // Check if we need to scroll
            if (terminal.current_y + terminal.line_height > terminal.max_y) {
                int16_t scroll_amount = terminal.line_height + terminal.line_spacing;
                terminal.display->scroll(0, -scroll_amount);
                terminal.current_y -= scroll_amount;
                terminal.display->fillRect(
                    terminal.margin_horizontal,
                    terminal.current_y,
                    terminal.content_width,
                    scroll_amount,
                    TFT_BLACK
                );
            }
        }
        // Advance to next line
        terminal.current_y += terminal.line_height + terminal.line_spacing;
        return;
    }

    // Process text in chunks that fit on one line (excluding trailing newline)
    while (pos < text_len) {
        // Find next newline or end of available chars for this line
        int16_t line_end = pos;
        int16_t chars_to_write = 0;

        // Look for newline or max chars, whichever comes first
        while (chars_to_write < max_chars && line_end < text_len && buffer[line_end] != '\n') {
            line_end++;
            chars_to_write++;
        }

        // Add line to buffer
        if (chars_to_write > 0) {
            add_line_to_buffer(&buffer[pos], chars_to_write);
        } else {
            add_line_to_buffer("", 0);
        }

        // Only update display if terminal is active
        if (terminal.is_active) {
            // Check if we need to scroll
            if (terminal.current_y + terminal.line_height > terminal.max_y) {
                // Scroll up by one line height + spacing
                int16_t scroll_amount = terminal.line_height + terminal.line_spacing;
                terminal.display->scroll(0, -scroll_amount);

                // Move cursor back up since we scrolled
                terminal.current_y -= scroll_amount;

                // Clear the bottom line where we'll write (respecting margins)
                terminal.display->fillRect(
                    terminal.margin_horizontal,
                    terminal.current_y,
                    terminal.content_width,
                    scroll_amount,
                    TFT_BLACK
                );
            }

            // Set cursor position and print text chunk (if any)
            if (chars_to_write > 0) {
                terminal.display->setCursor(terminal.x_offset, terminal.current_y);
                for (int16_t i = 0; i < chars_to_write; i++) {
                    terminal.display->print(buffer[pos + i]);
                }
            }
        }

        // Move to next line (with spacing)
        terminal.current_y += terminal.line_height + terminal.line_spacing;
        pos += chars_to_write;

        // If we hit a newline, skip it
        if (pos < text_len && buffer[pos] == '\n') {
            pos++;
        }
    }

    // If the original string ended with \n, add an extra blank line
    if (ends_with_newline) {
        // Add empty line to buffer
        add_line_to_buffer("", 0);

        // Only update display if terminal is active
        if (terminal.is_active) {
            // Check if we need to scroll for the blank line
            if (terminal.current_y + terminal.line_height > terminal.max_y) {
                int16_t scroll_amount = terminal.line_height + terminal.line_spacing;
                terminal.display->scroll(0, -scroll_amount);
                terminal.current_y -= scroll_amount;
                terminal.display->fillRect(
                    0,
                    terminal.current_y,
                    terminal.display->width(),
                    scroll_amount,
                    TFT_BLACK
                );
            }
        }
        // Advance cursor for the blank line (don't write anything)
        terminal.current_y += terminal.line_height + terminal.line_spacing;
    }
}

int16_t terminal_get_max_chars_per_line() {
    // Get the width of a single character
    // Using 'M' as a reference character (typically widest for monospace fonts)
    int16_t char_width = terminal.display->textWidth("M");

    // Calculate how many characters fit in the content width
    return terminal.content_width / char_width;
}

// Helper function to redraw terminal screen
static void draw_terminal_screen() {
    // Clear screen
    terminal.display->fillScreen(TFT_BLACK);

    // Calculate how many lines fit on the display
    int16_t available_height = terminal.max_y - terminal.y_offset;
    int16_t max_visible_lines = available_height / (terminal.line_height + terminal.line_spacing);

    // Determine how many lines to draw (minimum of buffer size and max visible)
    size_t lines_to_draw = terminal.line_buffer_count;
    if (lines_to_draw > max_visible_lines) {
        lines_to_draw = max_visible_lines;
    }

    // Calculate starting index in circular buffer (draw the most recent lines)
    size_t start_line_index;
    if (terminal.line_buffer_count <= TERMINAL_LINE_BUFFER_SIZE) {
        // Buffer hasn't wrapped yet
        if (terminal.line_buffer_count <= max_visible_lines) {
            start_line_index = 0;
        } else {
            start_line_index = terminal.line_buffer_count - max_visible_lines;
        }
    } else {
        // Buffer has wrapped, calculate from the end
        start_line_index = terminal.line_buffer_start;
        size_t skip_lines = terminal.line_buffer_count - lines_to_draw;
        start_line_index = (start_line_index + skip_lines) % TERMINAL_LINE_BUFFER_SIZE;
    }

    // Draw lines from buffer
    int16_t y_pos = terminal.y_offset;
    for (size_t i = 0; i < lines_to_draw; i++) {
        size_t buffer_index;
        if (terminal.line_buffer_count <= TERMINAL_LINE_BUFFER_SIZE) {
            buffer_index = start_line_index + i;
        } else {
            buffer_index = (start_line_index + i) % TERMINAL_LINE_BUFFER_SIZE;
        }

        draw_line_on_display(terminal.line_buffer[buffer_index], y_pos);
        y_pos += terminal.line_height + terminal.line_spacing;
    }

    // Set current_y to the position after the last drawn line
    terminal.current_y = y_pos;
    terminal.display->setCursor(terminal.x_offset, terminal.current_y);
}

void terminal_redraw() {
    // Mark terminal as active and redraw buffered content
    terminal.is_active = true;
    draw_terminal_screen();
}

void terminal_update() {
    // Currently does nothing - terminal_write() handles incremental updates
    // This function is available for future use (e.g., cursor blinking, status updates)
}

void terminal_deactivate() {
    // Mark terminal as inactive so terminal_write() won't draw to display
    terminal.is_active = false;
}
