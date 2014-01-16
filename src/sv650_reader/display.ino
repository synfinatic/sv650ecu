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

const char DISPPOS[4] = { 0xc0, 0xc2, 0xc4, 0xc6 };

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
display_char(int position, char digit)
{
    digitalWrite(CS,LOW);
    shiftOut(MOSI, CLK, LSBFIRST, DISPPOS[position]);
    shiftOut(MOSI, CLK, LSBFIRST, get_display_char(digit));
    digitalWrite(CS,HIGH);    
}

/*
 * Print error code to LED display from the table
 */
void 
print_led_error(char tps_adjust, int idx, int efi_alarm) {
   int i = 0;
   byte led_digits[4] = { 0xc0, 0xc2, 0xc4, 0xc6 };

   led.sendDigit(led_digits[0], tps_adjust);

   if (efi_alarm == 0) {
       // There is no alarm, but we got called (Dealer mode?) so 
       // return the "no error" code.  Calculate index of "no error":
       idx = sizeof(error_table) / sizeof(ECU_ERRORS) - 1;
   } else if (idx < 0) {
       // There is an alarm, but we don't know the code, so just return
       return;
   }

#ifdef DEBUG 
   serial_printf("printing error index: %d\n", idx);
#endif

   for (i = 1; i < 4; i++) {
       led.sendDigit(led_digits[i], error_table[idx].led[i-1]);
   } 
}

/*
 * Print water temp
 */
void 
print_led_temp() {
    unsigned int adc_value;
    unsigned int temp;
    int holding;
    char display[4];
#ifdef USE_CELCIUS
    float celcius;
#endif

    // temp is all of byte 0 and top two bits of byte 1
    adc_value = sbytes[0] << 2;
    adc_value += (sbytes[1] & 0xc0) >> 6;

    /* 
     * covert temp into a value fareinheit and then print it
     */
    display[3] = get_display_char('F');
    if (adc_value <= 42) {
        // temp == HI 
        display[0] = get_display_char('H');
        display[1] = get_display_char('I');
        display[2] = 0;
    } else if (adc_value >= 525) {
        // temp == LO (--- on stock display) 
        display[0] = get_display_char('L'); 
        display[1] = get_display_char('O'); 
        display[2] = 0;
    } else {
        // temp == XXXF
        temp = pgm_read_byte_near(temp_table + (adc_value - 42)) + 60;
#ifdef USE_CELCIUS
        // (F - 32) * 5/9 = C
        celcius = (float)(temp - 32) * ((float)5/9);
        temp = (unsigned int)celcius;
#endif
        serial_printf("temp is %u\n", temp);

        holding = temp % 10;
        display[2] = get_display_hex(holding);
        temp -= holding;
        if (temp > 199) {
            display[0] = get_display_hex(2);
            temp -= 200;
        } else if (temp > 99) {
            display[0] = get_display_hex(1);
            temp -= 100;
        } else {
            display[0] = 0;
        }

        display[1] = get_display_hex(temp / 10);

    }
    display_values(display[0], display[1], display[2], display[3], 0);
}

/*
 * Notify user of bad Temp condition
 */
void 
print_led_bad_temp() {
    char display[4] = { 'B', 'A', 'D', 'T' };  // BADT
#ifdef DEBUG
    serial_printf("print_led_bad_temp\n");
#endif
    display_chars(display[0], display[1], display[2], display[3]);
}

/*
 * Notify user of bad EFI condition
 */
void 
print_led_bad_efi() {
    char display[4] = { 'B', 'A', 'D', 'E' };  // BADE
#ifdef DEBUG
    serial_printf("print_led_bad_efi\n");
#endif
    display_chars(display[0], display[1], display[2], display[3]);
}

