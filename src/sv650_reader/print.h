/* 
 * Copyright 2013-2014 (c) Aaron Turner
 * This code is released under the GPLv3 license.  Please see the LICENSE file 
 * for details.
 *
 * More information is available here: https://github.com/synfinatic/sv650ecu
 */

#ifndef DISPLAY_H
#define DISPLAY_H

void print_led_error(char, int, int);
int print_led_ecu();
void print_led_bad_ecu();
void print_battery_voltage();

#endif
