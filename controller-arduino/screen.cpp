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
    //    lcd->drawBitmap(0,0,width,height,(const uint8_t*)buffer);
    lcd->switchFrameBufferTo((void*)buffer);
    buffer = (uint16_t*)lcd->getFrameBufferByIndex(which);
    which = (which==0)?1:0;
}
       
    
