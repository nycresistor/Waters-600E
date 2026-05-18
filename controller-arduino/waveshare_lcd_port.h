#ifndef __LCD_PORT_H
#define __LCD_PORT_H

#pragma once

#include <Arduino.h>
#include <esp_display_panel.hpp>

// I2C Pin define 
#define I2C_MASTER_NUM I2C_NUM_0 // I2C master number
#define I2C_MASTER_SDA_IO 8       // I2C data line
#define I2C_MASTER_SCL_IO 9       // I2C clock line

// Extend IO Pin define
#define TP_RST 1
#define LCD_BL 2
#define LCD_RST 3
#define SD_CS 4
#define USB_SEL 5     // USB select pin

#define LCD_NAME                        ST7262 // LCD model name
#define LCD_WIDTH                       (800) // LCD width in pixels
#define LCD_HEIGHT                      (480) // LCD height in pixels
#define LCD_COLOR_BITS                  (24)  // Color depth in bits
#define LCD_RGB_COLOR_BITS          (16)    // |      24      |      16       |

#define LCD_RGB_TIMING_FREQ_HZ          (16 * 1000 * 1000) // RGB timing frequency
#define LCD_RGB_TIMING_HPW              (4)   // Horizontal pulse width
#define LCD_RGB_TIMING_HBP              (8)   // Horizontal back porch
#define LCD_RGB_TIMING_HFP              (8)   // Horizontal front porch
#define LCD_RGB_TIMING_VPW              (4)   // Vertical pulse width
#define LCD_RGB_TIMING_VBP              (8)   // Vertical back porch
#define LCD_RGB_TIMING_VFP              (8)   // Vertical front porch
#define LCD_RGB_BOUNCE_BUFFER_SIZE  (LCD_WIDTH * 10)


#define LCD_RGB_IO_DISP            (-1)  // RGB display pin number
#define LCD_RGB_IO_VSYNC           (3)   // VSYNC pin number
#define LCD_RGB_IO_HSYNC           (46)  // HSYNC pin number
#define LCD_RGB_IO_DE              (5)   // Data enable pin number
#define LCD_RGB_IO_PCLK            (7)   // Pixel clock pin number
#define LCD_RGB_IO_DATA0           (14)  // RGB data pin 0
#define LCD_RGB_IO_DATA1           (38)  // RGB data pin 1
#define LCD_RGB_IO_DATA2           (18)  // RGB data pin 2
#define LCD_RGB_IO_DATA3           (17)  // RGB data pin 3
#define LCD_RGB_IO_DATA4           (10)  // RGB data pin 4
#define LCD_RGB_IO_DATA5           (39)  // RGB data pin 5
#define LCD_RGB_IO_DATA6           (0)   // RGB data pin 6
#define LCD_RGB_IO_DATA7           (45)  // RGB data pin 7
#define LCD_RGB_IO_DATA8           (48)  // RGB data pin 8
#define LCD_RGB_IO_DATA9           (47)  // RGB data pin 9
#define LCD_RGB_IO_DATA10          (21)  // RGB data pin 10
#define LCD_RGB_IO_DATA11          (1)   // RGB data pin 11
#define LCD_RGB_IO_DATA12          (2)   // RGB data pin 12
#define LCD_RGB_IO_DATA13          (42)  // RGB data pin 13
#define LCD_RGB_IO_DATA14          (41)  // RGB data pin 14
#define LCD_RGB_IO_DATA15          (40)  // RGB data pin 15
#define LCD_RST_IO                 (-1)  // Reset pin number
#define LCD_BL_IO            (-1)  // Backlight pin number
#define LCD_BL_ON_LEVEL           (1)   // Backlight ON level
#define LCD_BL_OFF_LEVEL !LCD_BL_ON_LEVEL // Backlight OFF level

#define LCD_ENABLE_PRINT_FPS            (0)
#define LCD_ENABLE_DRAW_FINISH_CALLBACK (0)

#define _LCD_CLASS(name, ...) LCD_##name(__VA_ARGS__)
#define LCD_CLASS(name, ...)  _LCD_CLASS(name, ##__VA_ARGS__)

#if LCD_ENABLE_PRINT_FPS
#define LCD_PRINT_FPS_PERIOD_MS         (1000)
#define LCD_PRINT_FPS_COUNT_MAX         (50)

#endif // LCD_ENABLE_PRINT_FPS

#endif // End of IO_PORT_H
