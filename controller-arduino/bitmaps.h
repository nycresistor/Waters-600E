#include <stdint.h>
#include "screen.h"

class RLEBitmap {
public:
    const uint16_t w, h, x, y;
private:
    const uint8_t* const data;
    const uint16_t dlen;
public:
    RLEBitmap(uint16_t w,uint16_t h,uint16_t x,uint16_t y,
	      const uint8_t* const data, uint16_t dlen);
    void put_at_alpha(Screen* s, uint16_t x, uint16_t y, uint16_t color);
    void put_at(Screen* s, uint16_t x, uint16_t y, uint16_t fg, uint16_t bg);
    void put_at_default_alpha(Screen* s, uint16_t color);
    void put_at_default(Screen* s, uint16_t fg, uint16_t bg);
};
