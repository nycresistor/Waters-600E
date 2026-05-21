#pragma once
#include <stdint.h>
#include <esp_display_panel.hpp>

class Screen {
public:
    Screen(esp_panel::drivers::LCD* lcd, uint16_t width, uint16_t height);
    const uint16_t width;
    const uint16_t height;
    const uint16_t line_offset;
    uint16_t* buffer;
    void flip();
private:
    uint8_t which;
    esp_panel::drivers::LCD* lcd;
};

