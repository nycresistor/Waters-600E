#include "fonts.h"

#include "waters_font.h"

FixedFont::FixedFont( const int16_t* enc_table, const uint8_t* font_data,
		  const uint8_t enc_table_size, const uint8_t cell_w,
		  const uint8_t cell_h ) :
    enc_table(enc_table), font_data(font_data),
    enc_table_size(enc_table_size), cell_w(cell_w),
    cell_h(cell_h) {}

FixedFont waters(font_enc_map, font_data, MAX_ENCODING+1,
		 8, 16);

inline void fill2x2(uint16_t* buf, uint16_t color, uint16_t off) {
    buf[0] = color; buf[1] = color; buf[off] = color; buf[off+1] = color;
}

bool FixedFont::put_char_at(Screen* screen, int16_t x, int16_t y, char c, CharAttr attribs) {
    if (x < 0 || x >= screen->width || y < 0 || y >= screen->height) return false;
    if (c >= enc_table_size) return false;
    int idx = enc_table[c];
    if (idx == -1) return false;
    uint16_t* buf = screen->buffer + (y*screen->line_offset) + x;
    if (attribs.double_size) {
	for (int i = 0; i < cell_h; i++) {
	    for (int j = 0; j < cell_w; j++) {
		if (font_data[idx] & (0x80>>j))
		    fill2x2(buf+j, attribs.fg, screen->line_offset);
		else if (!attribs.alpha_bg) fill2x2(buf+j, attribs.bg, screen->line_offset);
	    }
	    idx++;
	    buf += screen->line_offset*2;
	}
    } else {
	for (int i = 0; i < cell_h; i++) {
	    for (int j = 0; j < cell_w; j++) {
		if (font_data[idx] & (0x80>>j)) buf[j] = attribs.fg;
		else if (!attribs.alpha_bg) buf[j] = attribs.bg;
	    }
	    idx++;
	    buf += screen->line_offset;
	}
    }
    return true;
}
    
bool FixedFont::put_str_at(Screen* screen, int16_t x, int16_t y, char* s, CharAttr attribs) {
    if (x < 0 || x >= screen->width || y < 0 || y >= screen->height) return false;
    while (*s != '\0' && x < screen->width) {
	if (!put_char_at(screen,x,y,*s,attribs)) return false;
	s++;
	x+=attribs.double_size?(cell_w*2):cell_w;
    }
    return true;
}
