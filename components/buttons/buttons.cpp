#include "buttons.hpp"
#include "terminal.hpp"
#include <M5Unified.hpp>

void buttons_init() {
    // Button configuration can go here if needed
    // For example: M5.BtnA.setDebounceThresh(20);
}

void buttons_check() {
    // Check button states immediately after M5.update()
    // was* methods are edge-triggered and only return true once per event

    if (M5.BtnA.wasClicked()) {
        terminal_write("Button A clicked");
    }

    if (M5.BtnA.wasHold()) {
        terminal_write("Button A held");
    }

    if (M5.BtnB.wasClicked()) {
        terminal_write("Button B clicked");
    }

    if (M5.BtnC.wasClicked()) {
        terminal_write("Button C clicked");
    }

    if (M5.BtnPWR.wasClicked()) {
        terminal_write("Power button clicked");
    }
}
