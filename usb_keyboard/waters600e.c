/* Keyboard example for Teensy USB Development Board
 * http://www.pjrc.com/teensy/usb_keyboard.html
 * Copyright (c) 2008 PJRC.COM, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usb_keyboard.h"

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

/*****************
 Address pins - output
 Data pins - input
 * - confirmed
 A0 - D2*   D0 - F5*
 A1 - F6*   D1 - B3*
 A2 - F7*   D2 - F4*
 A3 - B6    D3 - B2
 A4 - B5    D4 - F1*
 A5 - D0*   D5 - B1
 A6 - D1*   D6 - F0
 A7 - B7    D7 - B0*
 *****************/

uint8_t keyState[8];

/****************
Keyboard matrix layout
    D6    D7    C6    B4    D3    B5    D1    F7
C7  3     6     9     -     .     -     RGT   -  
D2  2     5     8     CLR   0     ENT   HOM   DWN
D0  1     F5    F4    F3    F2    F1    4     7
F6  SET   UP    PEVT  PGRD  OGRD  ISOC  -     LFT
******************/

uint16_t keyMap[4*8] = {
  KEY_3, KEY_6, KEY_9, 0, KEY_PERIOD, 0, KEY_RIGHT, 0,
  KEY_2, KEY_5, KEY_8, KEY_DELETE, KEY_0, KEY_ENTER, KEY_HOME, KEY_DOWN,
  KEY_1, KEY_F5, KEY_F4, KEY_F3, KEY_F2, KEY_F1, KEY_4, KEY_7,
  KEY_F6, 0, KEY_F10, KEY_F9, KEY_F8, KEY_F7, KEY_UP, KEY_LEFT
};

void initState(void) {
  uint8_t idx;
  for (idx = 0; idx < 4; idx++) keyState[idx] = 0;
}

uint8_t key_count;
uint8_t key_touched;

void clearReport(void) {
  keyboard_modifier_keys = 0;
  for (int8_t i = 0; i < 6; i++) {
    keyboard_keys[i] = 0;
  }
  key_count = 0;
  key_touched = 0;
}

void addToReport(uint16_t key) {
  key_touched = 1;
  if (key > 0xff) {
    keyboard_modifier_keys |= key >> 8;
  } else {
    if (key_count < 6) {
      keyboard_keys[key_count++] = key & 0xff;
    }
  }
}

void endReport(void) {
  if (key_touched) {
    usb_keyboard_send();
  }
}

void doKeyState(uint8_t a, uint8_t d, uint8_t state) {
  uint8_t oldState = (keyState[a] & _BV(d)) != 0;
  uint8_t newState = state == 0;
  uint16_t key = keyMap[(a*8)+d];
  if (key == 0) return;
  if (oldState != newState) {
    key_touched = 1;
  }
  if (newState) {
    addToReport(key);
    keyState[a] |= _BV(d);
  } else {
    keyState[a] &= ~_BV(d);
  }
}

void setAddr(int8_t idx) {
  // set all addr lines hi-impedence with
  // pullups switched off
  PORTC |= _BV(7);
  PORTD |= _BV(2) | _BV(0);
  PORTF |= _BV(6);
  // set correct line low
  switch(idx) {
  case 0: 
    PORTC &= ~_BV(7);
    break;
  case 1: 
    PORTD &= ~_BV(2);
    break;
  case 2: 
    PORTD &= ~_BV(0);
    break;
  case 3: 
    PORTF &= ~_BV(6);
    break;
  }
}


void readCycle(int8_t idx) {
  setAddr(idx);
  _delay_ms(1);
  doKeyState(idx,0,PIND & _BV(6));
  doKeyState(idx,1,PIND & _BV(7));
  doKeyState(idx,2,PINC & _BV(6));
  doKeyState(idx,3,PINB & _BV(4));
  doKeyState(idx,4,PIND & _BV(3));
  doKeyState(idx,5,PINB & _BV(5));
  doKeyState(idx,6,PIND & _BV(1));
  doKeyState(idx,7,PINF & _BV(7));
  _delay_ms(1);
}

void initPins(void) {
  // All pins set to inputs
  DDRC = _BV(7); PORTC = _BV(7);
  DDRD = _BV(2) | _BV(0); PORTD = _BV(2) | _BV(0);
  DDRF = _BV(6); PORTF = _BV(6);
  // Pullups for input lines
  PORTB = _BV(4) | _BV(5);
  PORTC = _BV(6);
  PORTD = _BV(6) | _BV(7) | _BV(3) | _BV(1);
  PORTF = _BV(7);
}

int main(void)
{
	// set for 16 MHz clock
	CPU_PRESCALE(0);

	initState();
	initPins();
	TCCR0A &= 0x03;
	TCCR1A &= 0x03;

	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();
	while (!usb_configured()) /* wait */ ;

	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

	while (1) {
	  uint8_t idx;
	  clearReport();
	  for (idx = 0; idx < 4; idx++) {
	    readCycle(idx);
	  }
	  endReport();
	  // debouncing delay
	  _delay_ms(2);
	}
}


