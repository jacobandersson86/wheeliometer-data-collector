#include <M5Unified.h>
#include <stdio.h>
#include <terminal.hpp>

extern "C" {
    void app_main(void)
    {
        auto cfg = M5.config();
        M5.begin(cfg);

        terminal_init(&M5.Display);

        uint16_t i = 0;
        while (true) {
            terminal_write("Msg %d: This text is way too long to fit in one line", i);
            terminal_write("Hello\nWorld!\n");
            terminal_write("Hello\n");
            terminal_write("World");
            terminal_write("");
            M5.delay(500);
            i++;
            if (i > 3) {
                break;
            }
        }
    }

}


