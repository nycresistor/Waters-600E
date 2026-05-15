#include "waveshare_lcd_port.h"
#include "esp_memory_utils.h"

#include "waters_font.h"

using namespace esp_panel::drivers;
using namespace esp_panel::utils;

LCD *create_lcd_with_config(void);
LCD *create_lcd_without_config(void);

LCD* lcd;
bool put_char(uint16_t* buf, int lineoff, char c, uint16_t color) {
    if (c > MAX_ENCODING) return false;
    int idx = font_enc_map[c];
    if (idx == -1) return false;
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 8; j++) {
            if (font_data[idx] & (1<<j)) buf[j] = color;
        }
        idx++;
        buf += lineoff;
    }
    return true;
}
bool draw_square(int x, int y, int w, int h, uint16_t color)
{
    if (!lcd->isOverState(LCD::State::BEGIN)) return false;

    int bitspp = lcd->getFrameColorBits();

    auto y_coord_align = lcd->getBasicAttributes().basic_bus_spec.y_coord_align;
    int bpp = bitspp / 8;
    // Make sure the height is aligned to the `y_coord_align`
    //Serial.print(x); Serial.print(","); Serial.print(y); Serial.print(" bpp is"); Serial.println(bpp);

    /* Malloc memory for a single color bar */
    vector<uint16_t> buf_v(w*h);
    auto buf = buf_v.data();
    for (int i = 0; i < w*h; i++) buf[i] = color;
    put_char(buf, w, 'A' + (color %52), 0x0000);
    if (!lcd->drawBitmap(x, y, w, h, (uint8_t*)buf, -1)) Serial.println("drawbitmap oh no");
    return true;
}

void setup()
{
    Serial.begin(115200); // Initialize serial communication at 115200 baud rate
    Serial.println("RGB LCD example start"); // Print start message for RGB LCD example

    lcd = create_lcd_without_config();
    //auto lcd = create_lcd_without_config();
    // Configure bounce buffer to avoid screen drift
    auto bus = static_cast<BusRGB *>(lcd->getBus());
    bus->configRGB_BounceBufferSize(EXAMPLE_LCD_RGB_BOUNCE_BUFFER_SIZE); // Set bounce buffer to avoid screen drift

    lcd->init();
    // Attach a callback function which will be called when the Vsync signal is detected
    //lcd->attachRefreshFinishCallback(onLCD_RefreshFinishCallback);
    // Attach a callback function which will be called when every bitmap drawing is completed
    //lcd->attachDrawBitmapFinishCallback(onLCD_DrawFinishCallback);
    lcd->reset();
    assert(lcd->begin());
    if (lcd->getBasicAttributes().basic_bus_spec.isFunctionValid(LCD::BasicBusSpecification::FUNC_DISPLAY_ON_OFF)) {
        lcd->setDisplayOnOff(true);
    }

    Serial.println("Draw color bar from top left to bottom right, the order is B - G - R");
    //lcd->colorBarTest();

    Serial.println("RGB LCD example end"); // Print end message for RGB LCD example
}

int x = 0;
int y = 0;
int frame = 0;
const int blocksize = 40;
void loop()
{
    delay(10); 
    uint8_t shift = (x+y+(2*frame)) % 16;
    uint16_t color = 0xf000 >> shift | 0x00f0 << (16 - shift);
    if(!draw_square(x*blocksize,y*blocksize,blocksize,blocksize,color)) Serial.println("oh no");
    x++;
    if (x >= 800/blocksize) { x = 0; y++; if (y >= 480/blocksize) { y = 0; frame++; } }

}
