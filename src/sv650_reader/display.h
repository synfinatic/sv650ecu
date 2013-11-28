#ifndef DISPLAY_H
#define DISPLAY_H

byte get_display_char(char);
byte get_display_hex(int);
void display_chars(char, char, char, char);
void display_values(char, char, char, char);
void display_char(int, char);
void print_led_error(char, int);
void print_led_temp();
void print_led_bad_temp();
void print_led_bad_efi();


#endif
