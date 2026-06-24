#include "screen.h"

using namespace esp_panel::drivers;

Screen::Screen(LCD* lcd, uint16_t width, uint16_t height,
	       uint16_t xoff, uint16_t yoff) :
    width(width),
    height(height),
    xoff(xoff),
    yoff(yoff),
    rot(true),
    line_offset(lcd->getFrameWidth()),
    lcd_h(lcd->getFrameHeight()),
    lcd(lcd)
{
    buffer = (uint16_t*)lcd->getFrameBufferByIndex(1);
    which = 0;
}

void Screen::flip() {
    lcd->switchFrameBufferTo((void*)buffer);
    buffer = (uint16_t*)lcd->getFrameBufferByIndex(which);
    which = (which==0)?1:0;
}

void Screen::clear(uint16_t color) {
    draw_square(300,300,100,100,color);
    uint32_t last_px = lcd->getFrameHeight() * lcd->getFrameWidth();
    for (uint32_t i = 0; i < last_px; i++) buffer[i] = color;
}

void Screen::draw_square(int x, int y, int w, int h, uint16_t color)
{
    for (int i = 0; i < h; i++) {
	for (int j = 0; j < w; j++) set_px(x+j,y+i,color);
    }	
}

    
