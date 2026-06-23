#include <WiFi.h>
#include "ESP32MQTTClient.h"
#include "Wire.h"

#include "credentials.h"

#include "waveshare_lcd_port.h"
#include "esp_memory_utils.h"

#include "fonts.h"
#include "bitmaps.h"
#include "keypad.h"

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


/*
| Column | 1     | 4  | 5    | 6     | 7     | 8     | 11    | 12   |
|--------|-------|----|------|-------|-------|-------|-------|------|
| 3      | 3     | 6  | 9    |       | .     |       | Right |      |
| 9      | 2     | 5  | 8    | Clear | 0     | Enter | Home  | Down |
| 13     | 1     | F5 | F4   | F3    | F2    | F1    | 4     | 7    |
| 14     | SetUp |    | PEvt | PGrad | OGrad | Isoc  | Up    | Left |

| Connector | Port | Pin no |
|-----
*/


void gpio_ext_set_col(uint8_t col) {
    // Columns are
    const uint8_t col_pins[KP_COLUMNS] = {
	6,3,1,9 };
    // set all to input
    gpio_ext_wr_reg(0x04, 0xff);
    gpio_ext_wr_reg(0x05, 0xff);
    if (col >= KP_COLUMNS) return;
    uint8_t which = col_pins[col];
    if (which < 8) {
	gpio_ext_wr_reg(0x04, 0xff ^ (1 << which));
	gpio_ext_wr_reg(0x02, 1 << which);
    } else {
	which -= 8;
	gpio_ext_wr_reg(0x05, 0xff ^ (1 << which));
	gpio_ext_wr_reg(0x03, 1 << which);
    }
}

#define GET_BIT(FROM,FROM_BIT) \
    (( FROM >> FROM_BIT ) & 0x01 )

uint8_t gpio_ext_get_rows() {
    uint8_t rows = 0;
    uint8_t val = gpio_ext_rd_reg(0x00);
    rows |= GET_BIT(val,7);
    rows |= GET_BIT(val,5) << 2;
    rows |= GET_BIT(val,4) << 4;
    rows |= GET_BIT(val,2) << 6;
    val = gpio_ext_rd_reg(0x01);
    rows |= GET_BIT(val,6) << 1;
    rows |= GET_BIT(val,5) << 3;
    rows |= GET_BIT(val,4) << 5;
    rows |= GET_BIT(val,2) << 7;
    return rows;
}

bool init_gpio_ext() {
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
    // Set all to input and PORT 0 to push-pull
    gpio_ext_wr_reg(0x04, 0xff);
    gpio_ext_wr_reg(0x05, 0xff);
    gpio_ext_wr_reg(0x11, 0x10);
    
    return false;
}

void setup()
{
    Serial.begin(115200); // Initialize serial communication at 115200 baud rate
    Wire.begin(8,9); // SDA 8 SCL 9

    lcd = create_lcd();
    //auto lcd = create_lcd_without_config();
    // Configure bounce buffer to avoid screen drift
    auto bus = static_cast<BusRGB *>(lcd->getBus());
    bus->configRGB_BounceBufferSize(2*LCD_RGB_BOUNCE_BUFFER_SIZE); // Set bounce buffer to avoid screen drift
    lcd->configFrameBufferNumber(2);
    lcd->init();
    //lcd->attachRefreshFinishCallback(onLCD_RefreshFinishCallback);
    //lcd->attachDrawBitmapFinishCallback(onLCD_DrawFinishCallback);
    lcd->reset();
    assert(lcd->begin());
    if (lcd->getBasicAttributes().basic_bus_spec.isFunctionValid(LCD::BasicBusSpecification::FUNC_DISPLAY_ON_OFF)) {
        lcd->setDisplayOnOff(true);
    }
    scr = new Screen(lcd,LOGICAL_WIDTH,LCD_HEIGHT);

    init_gpio_ext();

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
    //waters.put_str_at(scr, 200, 200, "This is not really happening.", attr);
    attr.double_size = true;
    attr.alpha_bg = true;
    attr.fg = 0x000f;
    //waters.put_str_at(scr, 100, 100, "This is not real", attr);
    //SPLASH_bitmap.put_at_default_alpha(scr, 0xfa00);
    // Scan buttons
    gpio_ext_set_col(3);
    String rowstr = String("Row: ") + String(gpio_ext_get_rows(),HEX);
    waters.put_str_at(scr, 0, 320, rowstr.c_str(), (CharAttr){ .alpha_bg = false, .double_size = false,.fg = 0xffe7, .bg = 0x000f });
    String framestr = String(frame,HEX);
    waters.put_str_at(scr, 0, 350, framestr.c_str(), (CharAttr){ .alpha_bg = false, .double_size = false,.fg = 0xffe7, .bg = 0x000f });
    show_console();
    scr->flip();
}
