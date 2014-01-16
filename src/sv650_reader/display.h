/* 
 * Copyright 2013-2014 (c) Aaron Turner
 * This code is released under the GPLv3 license.  Please see the LICENSE file 
 * for details.
 *
 * More information is available here: https://github.com/synfinatic/sv650ecu
 */

#ifndef DISPLAY_H
#define DISPLAY_H

byte get_display_char(char);
byte get_display_hex(int);
void display_chars(char, char, char, char);
void display_values_colon(char, char, char, char, int);
void display_values(char, char, char, char);
void display_char(int, char);
void print_led_error(char, int);
void print_led_temp();
void print_led_bad_temp();
void print_led_bad_efi();


#endif
