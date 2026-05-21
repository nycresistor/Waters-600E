#include <WiFi.h>
#include "ESP32MQTTClient.h"

#include "credentials.h"

#include "waveshare_lcd_port.h"
#include "esp_memory_utils.h"

#include "fonts.h"
#include "bitmaps.h"

extern RLEBitmap SPLASH_bitmap;

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
    Screen scr(LOGICAL_WIDTH, LCD_HEIGHT,
	       (uint16_t*)lcd->getFrameBufferByIndex(0), LCD_WIDTH);
    SPLASH_bitmap.put_at_default_alpha(&scr, 0xfa00);
    const char* const* ssids_iter = ssids;
    while (*ssids_iter != nullptr) {
	Serial.print("Trying "); Serial.println(ssids_iter[0]);
        WiFi.begin(ssids_iter[0], ssids_iter[1]);
	int16_t timeout = 6000;
	while (true) {
	    auto status = WiFi.status();
	    if (status == WL_CONNECTED) {
		ssids_iter = nullptr;
		break;
	    } else if (status == WL_CONNECT_FAILED || timeout <= 0) {
		ssids_iter += 2;
		break;
	    }
	    delay(300);
	    timeout -= 300;
	    Serial.print(".");
	}
    }
    if (WiFi.status() == WL_CONNECTED) Serial.println("Connected to network");
    else Serial.println("Connection failed, giving up.");
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
    SPLASH_bitmap.put_at_default_alpha(&scr, 0xfa00);
}
