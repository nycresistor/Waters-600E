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
Screen* scr;

bool draw_square(int x, int y, int w, int h, uint16_t color)
{
    int bitspp = lcd->getFrameColorBits();
    uint16_t* fb = scr->buffer;
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
    bus->configRGB_BounceBufferSize(2*LCD_RGB_BOUNCE_BUFFER_SIZE); // Set bounce buffer to avoid screen drift
    lcd->configFrameBufferNumber(2);
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
    scr = new Screen(lcd,LOGICAL_WIDTH,LCD_HEIGHT);
    SPLASH_bitmap.put_at_default_alpha(scr, 0xfa00);
    scr->flip();
    const char* const* ssids_iter = ssids;
    while (ssids_iter != nullptr && *ssids_iter != nullptr) {
	Serial.print("Trying "); Serial.println(ssids_iter[0]);
        WiFi.begin(ssids_iter[0], ssids_iter[1]);
	int16_t timeout = 15000;
	while (true) {
	    auto status = WiFi.status();
	    if (status == WL_CONNECTED) {
		ssids_iter = nullptr;
		break;
	    } else if (status == WL_CONNECT_FAILED || timeout <= 0) {
		WiFi.disconnect();
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

int frame = 0;
const int blocksize = 40;
void loop()
{
    delay(100); 
    for (int by = 0; by < LOGICAL_HEIGHT/blocksize; by++) {
	for (int bx = 0; bx < LOGICAL_WIDTH/blocksize; bx++) {
	    uint8_t shift = (bx+by+(frame)) % 16;
	    uint16_t color = 0xf000 >> shift | 0x00f0 << (16 - shift);
	    draw_square(bx*blocksize,by*blocksize,blocksize,blocksize,color);
	}
    }
    frame++;

    CharAttr attr = { .alpha_bg = false, .double_size = false,
		      .fg = 0xffe7, .bg = 0x000f };
    waters.put_str_at(scr, 200, 200, "This is not really happening.", attr);
    attr.double_size = true;
    attr.alpha_bg = true;
    attr.fg = 0x000f;
    waters.put_str_at(scr, 100, 100, "This is not real", attr);
    SPLASH_bitmap.put_at_default_alpha(scr, 0xfa00);
    scr->flip();
}
