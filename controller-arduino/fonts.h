
#include <stdint.h>
#include "screen.h"

typedef struct {
    bool alpha_bg;
    bool double_size;
    uint16_t fg, bg;
} CharAttr;

class FixedFont {
private:
    const int16_t* enc_table;
    const uint8_t* font_data;
    const uint8_t enc_table_size;
public:
    FixedFont( const int16_t* enc_table, const uint8_t* font_data,
	       const uint8_t enc_table_size, const uint8_t cell_w,
	       const uint8_t cell_h );
    const uint8_t cell_w;
    const uint8_t cell_h;
    bool put_char_at(Screen* screen, int16_t x, int16_t y, char c, CharAttr attribs);
    bool put_str_at(Screen* screen, int16_t x, int16_t y, char* s, CharAttr attribs);
};

extern FixedFont waters;
