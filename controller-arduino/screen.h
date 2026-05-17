#include <stdint.h>

class Screen {
 public:
    Screen(const uint16_t width, const uint16_t height,
	   uint16_t* buffer) :
	width(width), height(height), buffer(buffer) {}
    const uint16_t width;
    const uint16_t height;
    uint16_t* buffer;
};

