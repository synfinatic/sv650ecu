/* 
 * Copyright 2013-2014 (c) Aaron Turner
 * This code is released under the GPLv3 license.  Please see the LICENSE file 
 * for details.
 *
 * More information is available here: https://github.com/synfinatic/sv650ecu
 */

#ifndef UTILS_H
#define UTILS_H

void clear_buf();
void serial_printf(const char *fmt, ... );
void dbg_serial_printf(const char *fmt, ... );
char *ftoa(char *a, double f, int precision);
unsigned int check_csum();
int parse_ecu_byte(byte data, unsigned long delta);
void parse_message();


#define GOOD_CSUM 0xffff
#endif
