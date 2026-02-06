#include <M5Unified.h>
#include <stdio.h>
#include <terminal.hpp>
#include <buttons.hpp>
#include <rtc.hpp>

extern "C" {
    void app_main(void)
    {
        auto cfg = M5.config();
        M5.begin(cfg);

        buttons_init();

        terminal_init(&M5.Display);
        terminal_write("Wheeliometer startup complete");

        // Initialize RTC
        // Use RTC_MODE_COMPILE_TIME for automatic compile-time setting
        // Use RTC_MODE_SERIAL to set time via serial with set_rtc_time.py
        rtc_init(RTC_MODE_SERIAL);

        while (true) {
            M5.delay(1);  // Give time to other tasks and feed watchdog
            M5.update();
            buttons_check();
            rtc_check_serial();  // Check for serial commands
        }
    }

}


