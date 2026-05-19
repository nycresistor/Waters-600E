#include <stdint.h>

const int KP_COLUMNS = 4;
const int KP_ROWS = 8;

enum {
    // 0-9 are the corresponding numbers
    KP_PERIOD = 10,
    KP_CLEAR,
    KP_ENTER,
    
    KP_UP,
    KP_LEFT,
    KP_RIGHT,
    KP_DOWN,
    KP_HOME,
    
    KP_F1,
    KP_F2,
    KP_F3,
    KP_F4,
    KP_F5,

    KP_SET_UP,
    KP_ISOCRATIC,
    KP_OPER_GRAD,
    KP_PROG_GRAD,
    KP_PROG_EVENT,

    KP_LAST
};

int decode_key(int col, int row);
