#pragma once
#include <stdint.h>

class Screen {
 public:
    Screen(const uint16_t width, const uint16_t height,
	   uint16_t* buffer, const uint16_t line_offset) :
	width(width), height(height), buffer(buffer), line_offset(line_offset) {}
    const uint16_t width;
    const uint16_t height;
    const uint16_t line_offset;
    uint16_t* buffer;
};

