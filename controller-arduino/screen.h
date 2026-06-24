#pragma once
#include <stdint.h>
#include <esp_display_panel.hpp>

class Screen {
public:
    Screen(esp_panel::drivers::LCD* lcd, uint16_t width, uint16_t height, uint16_t xoff = 0, uint16_t yoff = 0);
    const uint16_t width;
    const uint16_t height;
    const uint16_t xoff;
    const uint16_t yoff;
    const uint16_t line_offset;
    const uint16_t lcd_h;
    const bool rot;
    uint16_t* buffer;
    void clear(uint16_t color);
    void flip();
    void draw_square(int x, int y, int w, int h, uint16_t color);
    inline void set_px(int x, int y, uint16_t color);
private:
    uint8_t which;
    esp_panel::drivers::LCD* lcd;
};

inline void Screen::set_px(int x, int y, uint16_t color) {
    uint16_t* b = buffer;
    if (rot) {
	x = (line_offset - 1) - x; y = (lcd_h - 1) - y;
	b += (line_offset * (y-yoff)) - xoff + x;
    } else {
	b += (line_offset * (y+yoff)) + xoff + x;
    }
    *b = color;
}
