#include "waveshare_lcd_port.h"
#include "esp_memory_utils.h"

#include "fonts.h"

using namespace esp_panel::drivers;
using namespace esp_panel::utils;

#define LOGICAL_WIDTH 640
#define LOGICAL_HEIGHT 480

#define TERMINAL_WIDTH (LOGICAL_WIDTH/8)
#define TERMINAL_HEIGHT (LOGICAL_HEIGHT/8)

LCD *create_lcd();

LCD* lcd;

bool draw_square(int x, int y, int w, int h, uint16_t color)
{
    if (!lcd->isOverState(LCD::State::BEGIN)) return false;

    int bitspp = lcd->getFrameColorBits();
    int bpp = bitspp / 8;
    uint16_t* fb = (uint16_t*)lcd->getFrameBufferByIndex(0);
    fb += y*LCD_WIDTH + x;
    for (int i = 0; i < h; i++) {
	for (int j = 0; j < w; j++) fb[j] = color;
	fb += LCD_WIDTH;
    }
	
    return true;
}

void setup()
{
    Serial.begin(115200); // Initialize serial communication at 115200 baud rate

    lcd = create_lcd();
    //auto lcd = create_lcd_without_config();
    // Configure bounce buffer to avoid screen drift
    auto bus = static_cast<BusRGB *>(lcd->getBus());
    bus->configRGB_BounceBufferSize(LCD_RGB_BOUNCE_BUFFER_SIZE); // Set bounce buffer to avoid screen drift

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

const char* lorip = "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.";

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
    int ty = (frame+y+x/2)%TERMINAL_HEIGHT;
    //put_str_pos(0,ty,lorip + ty, 0x7000);
    x++;
    if (x >= LOGICAL_WIDTH/blocksize) { x = 0; y++; if (y >= LOGICAL_HEIGHT/blocksize) { y = 0; frame++; } }

    Screen scr(LOGICAL_WIDTH, LCD_HEIGHT,
	       (uint16_t*)lcd->getFrameBufferByIndex(0), LCD_WIDTH);
    CharAttr attr = { .alpha_bg = false, .double_size = false,
		      .fg = 0xffe7, .bg = 0x000f };
    waters.put_str_at(&scr, 200, 200, "This is not really happening.", attr);
    attr.double_size = true;
    attr.alpha_bg = true;
    attr.fg = 0x000f;
    waters.put_str_at(&scr, 100, 100, "This is not real", attr);

}
