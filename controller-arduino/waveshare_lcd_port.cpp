#include "waveshare_lcd_port.h"

using namespace esp_panel::drivers;

LCD *create_lcd_without_config(void)
{
    BusRGB *bus = new BusRGB(
#if LCD_RGB_DATA_WIDTH == 8
        /* 8-bit RGB IOs */
        LCD_RGB_IO_DATA0, LCD_RGB_IO_DATA1, LCD_RGB_IO_DATA2, LCD_RGB_IO_DATA3,
        LCD_RGB_IO_DATA4, LCD_RGB_IO_DATA5, LCD_RGB_IO_DATA6, LCD_RGB_IO_DATA7,
        LCD_RGB_IO_HSYNC, LCD_RGB_IO_VSYNC, LCD_RGB_IO_PCLK, LCD_RGB_IO_DE,
        LCD_RGB_IO_DISP,
        /* RGB timings */
        LCD_RGB_TIMING_FREQ_HZ, LCD_WIDTH, LCD_HEIGHT,
        LCD_RGB_TIMING_HPW, LCD_RGB_TIMING_HBP, LCD_RGB_TIMING_HFP,
        LCD_RGB_TIMING_VPW, LCD_RGB_TIMING_VBP, LCD_RGB_TIMING_VFP
#elif LCD_RGB_DATA_WIDTH == 16
        /* 16-bit RGB IOs */
        LCD_RGB_IO_DATA0, LCD_RGB_IO_DATA1, LCD_RGB_IO_DATA2, LCD_RGB_IO_DATA3,
        LCD_RGB_IO_DATA4, LCD_RGB_IO_DATA5, LCD_RGB_IO_DATA6, LCD_RGB_IO_DATA7,
        LCD_RGB_IO_DATA8, LCD_RGB_IO_DATA9, LCD_RGB_IO_DATA10, LCD_RGB_IO_DATA11,
        LCD_RGB_IO_DATA12, LCD_RGB_IO_DATA13, LCD_RGB_IO_DATA14, LCD_RGB_IO_DATA15,
        LCD_RGB_IO_HSYNC, LCD_RGB_IO_VSYNC, LCD_RGB_IO_PCLK, LCD_RGB_IO_DE,
        LCD_RGB_IO_DISP,
        /* RGB timings */
        LCD_RGB_TIMING_FREQ_HZ, LCD_WIDTH, LCD_HEIGHT,
        LCD_RGB_TIMING_HPW, LCD_RGB_TIMING_HBP, LCD_RGB_TIMING_HFP,
        LCD_RGB_TIMING_VPW, LCD_RGB_TIMING_VBP, LCD_RGB_TIMING_VFP
#endif
    );

    /**
     * Take `ST7262` as an example, the following is the actual code after macro expansion:
     *      LCD_ST7262(bus, 24, -1);
     */
    return new LCD_CLASS(
        LCD_NAME, bus, LCD_WIDTH, LCD_HEIGHT, LCD_COLOR_BITS, LCD_RST_IO
    );
}

#if LCD_ENABLE_PRINT_FPS

DRAM_ATTR int frame_count = 0;
DRAM_ATTR int fps = 0;
DRAM_ATTR long start_time = 0;

IRAM_ATTR bool onLCD_RefreshFinishCallback(void *user_data)
{
    if (start_time == 0) {
        start_time = millis();

        return false;
    }

    frame_count++;
    if (frame_count >= LCD_PRINT_FPS_COUNT_MAX) {
        fps = LCD_PRINT_FPS_COUNT_MAX * 1000 / (millis() - start_time);
        esp_rom_printf("LCD FPS: %d\n", fps);
        frame_count = 0;
        start_time = millis();
    }

    return false;
}
#endif // LCD_ENABLE_PRINT_FPS

#if LCD_ENABLE_DRAW_FINISH_CALLBACK
IRAM_ATTR bool onLCD_DrawFinishCallback(void *user_data)
{
    esp_rom_printf("LCD draw finish callback\n");

    return false;
}
#endif

