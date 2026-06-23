#include <WiFi.h>
#include "ESP32MQTTClient.h"
#include "Wire.h"

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
Screen* scr = nullptr;

const int C_W = 60;
const int C_H = 15;
char console[C_H][C_W];
int cur_line = 0;

const int GPIO_EXT_ADDR = 0x58; // base address of aw9523b

    
void console_msg(const char* txt) {
    int x;
    for (x = 0; x < C_W; x++) {
	console[cur_line][x] = *txt;
	if (*txt != 0) txt++;
    }
    cur_line = (cur_line + 1) % C_H;
}

void show_console() {
    CharAttr attr = { .alpha_bg = false, .double_size = false,
		      .fg = 0xffe7, .bg = 0x000f };
    for (int y = 0; y < C_H; y++)
	waters.put_str_at(scr,0,waters.cell_h*y, console[y], attr);
}

void print_console(const char* txt) {
    if (scr == nullptr) return;
    console_msg(txt); show_console();
    scr->flip();
}


void gpio_ext_wr_reg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(GPIO_EXT_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t gpio_ext_rd_reg(uint8_t reg) {
    Wire.beginTransmission(GPIO_EXT_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(GPIO_EXT_ADDR,1);
    return Wire.read();
}

bool init_gpio_ext() {
    Wire.begin(8,9); // SDA 8 SCL 9
    Wire.beginTransmission(GPIO_EXT_ADDR);
    if (Wire.endTransmission() == 0) {
	print_console("Contacted GPIO extender");
    } else {
	print_console("Could not contact GPIO extender");
	return false;
    }
    if (gpio_ext_rd_reg(0x10) == 0x23) {
	print_console("GPIO extender ID OK");
	return true;
    } else {
	print_console("GPIO extender ID wrong");
    }
    return false;
}

void setup()
{
    Serial.begin(115200); // Initialize serial communication at 115200 baud rate
    Serial.println("We are now beginning our investigation");
    init_gpio_ext();

    Serial.println("Wire library initialized.");

    lcd = create_lcd();

    Serial.println("LCD created.");
    //auto lcd = create_lcd_without_config();
    // Configure bounce buffer to avoid screen drift
    auto bus = static_cast<BusRGB *>(lcd->getBus());
    bus->configRGB_BounceBufferSize(2*LCD_RGB_BOUNCE_BUFFER_SIZE); // Set bounce buffer to avoid screen drift
    lcd->configFrameBufferNumber(2);
    Serial.println("LCD configured.");
    lcd->init();
    Serial.println("LCD init.");
    // Attach a callback function which will be called when the Vsync signal is detected
    //lcd->attachRefreshFinishCallback(onLCD_RefreshFinishCallback);
    // Attach a callback function which will be called when every bitmap drawing is completed
    //lcd->attachDrawBitmapFinishCallback(onLCD_DrawFinishCallback);
    lcd->reset();
    Serial.println("LCD reset.");
    assert(lcd->begin());
    Serial.println("LCD started.");
    if (lcd->getBasicAttributes().basic_bus_spec.isFunctionValid(LCD::BasicBusSpecification::FUNC_DISPLAY_ON_OFF)) {
        lcd->setDisplayOnOff(true);
    }
    scr = new Screen(lcd,LOGICAL_WIDTH,LCD_HEIGHT);
    SPLASH_bitmap.put_at_default_alpha(scr, 0xfa00);
    scr->flip();
    SPLASH_bitmap.put_at_default_alpha(scr, 0xfa00);
    print_console("Booting...");
    const char* const* ssids_iter = ssids;
    while (ssids_iter != nullptr && *ssids_iter != nullptr) {
	print_console("Trying:");
	print_console(ssids_iter[0]);
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
    if (WiFi.status() == WL_CONNECTED) print_console("Connected to network");
    else print_console("Connection failed, giving up.");
}

int frame = 0;
const int blocksize = 40;

void loop()
{
    delay(100);
    /*
    for (int by = 0; by < LOGICAL_HEIGHT/blocksize; by++) {
	for (int bx = 0; bx < LOGICAL_WIDTH/blocksize; bx++) {
	    uint8_t shift = (bx+by+(frame)) % 16;
	    uint16_t color = 0xf000 >> shift | 0x00f0 << (16 - shift);
	    scr->draw_square(bx*blocksize,by*blocksize,blocksize,blocksize,color);
	}
    }
    */ // This looks great but it's distracting for now
    frame++;

    CharAttr attr = { .alpha_bg = false, .double_size = false,
		      .fg = 0xffe7, .bg = 0x000f };
    waters.put_str_at(scr, 200, 200, "This is not really happening.", attr);
    attr.double_size = true;
    attr.alpha_bg = true;
    attr.fg = 0x000f;
    waters.put_str_at(scr, 100, 100, "This is not real", attr);
    SPLASH_bitmap.put_at_default_alpha(scr, 0xfa00);
    // Scan buttons
    show_console();
    scr->flip();
}
