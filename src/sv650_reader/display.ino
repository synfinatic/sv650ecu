/* 
 * Copyright 2013-2014 (c) Aaron Turner
 * This code is released under the GPLv3 license.  Please see the LICENSE file 
 * for details.
 *
 * More information is available here: https://github.com/synfinatic/sv650ecu
 */

/*
 * Functions for writing to the Oasis 4 char 7 segment display
 */

#include "sv650_reader.h"

const byte DISPPOS[4] = { 0xc0, 0xc2, 0xc4, 0xc6 };
extern bool efi_alarm, bad_efi, low_batt;

/*
 * Gets the LCD value for a given ASCII char
 */
byte 
get_display_char(char val)
{
    return pgm_read_byte_near(display_table + int(val));
}

/*
 * Gets the LCD value for a given byte value
 */
byte 
get_display_hex(int val)
{
    return pgm_read_byte_near(display_table + val + 0x30);
}

/*
 * Prints 4 characters on the 7 segment display
 */
void 
display_chars(char digit1, char digit2, char digit3, char digit4)
{

    digitalWrite(CS,LOW);
    shiftOut(MOSI, CLK, LSBFIRST, DISPPOS[0]);
    shiftOut(MOSI, CLK, LSBFIRST, get_display_char(digit1));
    shiftOut(MOSI, CLK, LSBFIRST, DISPPOS[1]);
    shiftOut(MOSI, CLK, LSBFIRST, get_display_char(digit2));
    shiftOut(MOSI, CLK, LSBFIRST, DISPPOS[2]);
    shiftOut(MOSI, CLK, LSBFIRST, get_display_char(digit3));
    shiftOut(MOSI, CLK, LSBFIRST, DISPPOS[3]);
    shiftOut(MOSI, CLK, LSBFIRST, get_display_char(digit4));
    digitalWrite(CS,HIGH);    
}

/*
 *  Prints 4 hex characters on the 7 segment display with colon
 *  colon & 0x01 == high colon bit 
 *  colon & 0x02 == low colon bit
 */
void 
display_values(char digit1, char digit2, char digit3, char digit4, int colon)
{

    digitalWrite(CS,LOW);
    shiftOut(MOSI, CLK, LSBFIRST, DISPPOS[0]);
    if (colon & 0x01) {
        shiftOut(MOSI, CLK, LSBFIRST, digit1 | 0x80);
    } else {
        shiftOut(MOSI, CLK, LSBFIRST, digit1);
    }
    shiftOut(MOSI, CLK, LSBFIRST, DISPPOS[1]);
    if (colon & 0x02) {
        shiftOut(MOSI, CLK, LSBFIRST, digit2 | 0x80);
    } else {
        shiftOut(MOSI, CLK, LSBFIRST, digit2);
    }
    shiftOut(MOSI, CLK, LSBFIRST, DISPPOS[2]);
    shiftOut(MOSI, CLK, LSBFIRST, digit3);
    shiftOut(MOSI, CLK, LSBFIRST, DISPPOS[3]);
    shiftOut(MOSI, CLK, LSBFIRST, digit4);
    digitalWrite(CS,HIGH);    
}

/*
 * Prints a single character at the given position on the 7 segment display
 */
void 
display_char(int position, byte digit)
{
    digitalWrite(CS,LOW);
    shiftOut(MOSI, CLK, LSBFIRST, DISPPOS[position]);
    shiftOut(MOSI, CLK, LSBFIRST, get_display_char(digit));
    digitalWrite(CS,HIGH);    
}

