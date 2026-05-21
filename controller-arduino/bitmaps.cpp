#include "bitmaps.h"

RLEBitmap::RLEBitmap(uint16_t w,uint16_t h,uint16_t x,uint16_t y,
		     const uint8_t* const data, uint16_t dlen) :
    w(w), h(h), x(x), y(y), data(data), dlen(dlen) {}

    
void RLEBitmap::put_at_alpha(Screen* s, uint16_t x, uint16_t y, uint16_t color) {
    uint16_t dx = 0;
    int16_t partial = -1;
    bool pfg = false;
    uint16_t* buf = s->buffer;
    buf += (y*s->line_offset + x);
    uint16_t bh = h;
    for (uint16_t idx  = 0; idx < dlen; idx++) {
	uint16_t rl;
	// unpack run length
	if (partial != -1) {
	    rl = partial << 8 | data[idx];
	    partial = -1;
	} else if ((data[idx] & 0x80) == 0) {
	    partial = data[idx];
	    continue;
	} else rl = data[idx] & 0x7f;
	while (rl > 0) {
	    if (pfg)
		buf[dx] = color;
	    dx++;
	    if (dx >= w) {
		dx = 0;
		bh--;
		buf += s->line_offset;
	    }
	    rl--;
	}
	pfg = !pfg;
    }
    while (bh > 0) {
	while (dx < w) {
	    if (pfg) buf[dx++] = color;
	}
	dx = 0;
	bh--;
	buf += s->line_offset;
    }
    
}

    
void RLEBitmap::put_at(Screen* s, uint16_t x, uint16_t y, uint16_t fg, uint16_t bg) {
    put_at_alpha(s, x, y, fg);
}

void RLEBitmap::put_at_default_alpha(Screen* s, uint16_t color) {
    put_at_alpha(s, x, y, color);
}

void RLEBitmap::put_at_default(Screen* s, uint16_t fg, uint16_t bg) {
    put_at(s, x, y, fg, bg);
}

