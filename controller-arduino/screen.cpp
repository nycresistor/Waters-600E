#include "screen.h"

using namespace esp_panel::drivers;

Screen::Screen(LCD* lcd, uint16_t width, uint16_t height) :
    width(width),
    height(height),
    line_offset(lcd->getFrameWidth()),
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

void Screen::draw_square(int x, int y, int w, int h, uint16_t color)
{
    uint16_t* fb = buffer;
    fb += y*line_offset + x;
    for (int i = 0; i < h; i++) {
	for (int j = 0; j < w; j++) fb[j] = color;
	fb += line_offset;
    }	
}

    
