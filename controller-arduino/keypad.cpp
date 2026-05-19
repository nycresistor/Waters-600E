#include "keypad.h"

/*
| Column | 1     | 4  | 5    | 6     | 7     | 8     | 11    | 12   |
|--------|-------|----|------|-------|-------|-------|-------|------|
| 3      | 3     | 6  | 9    |       | .     |       | Right |      |
| 9      | 2     | 5  | 8    | Clear | 0     | Enter | Home  | Down |
| 13     | 1     | F5 | F4   | F3    | F2    | F1    | 4     | 7    |
| 14     | SetUp |    | PEvt | PGrad | OGrad | Isoc  | Up    | Left |
*/

const int map[KP_COLUMNS*KP_ROWS] = {
    3, 6, 9, -1, KP_PERIOD, -1, KP_RIGHT, -1,
    2, 5, 8, KP_CLEAR, 0, KP_ENTER, KP_HOME, KP_DOWN,
    1, KP_F5, KP_F4, KP_F3, KP_F2, KP_F1, 4, 7,
    KP_SET_UP, -1, KP_PROG_EVENT, KP_PROG_GRAD, KP_OPER_GRAD, KP_ISOCRATIC, KP_UP, KP_LEFT };

int decode_key(int col, int row) {
    if (col < 0 || col > KP_COLUMNS) return -1;
    if (row < 0 || col > KP_ROWS) return -1;
    return map[row + col*KP_ROWS];
}

    
    
    
