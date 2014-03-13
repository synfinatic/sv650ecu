/* 
 * Copyright 2013-2014 (c) Aaron Turner
 * This code is released under the GPLv3 license.  Please see the LICENSE file 
 * for details.
 *
 * More information is available here: https://github.com/synfinatic/sv650ecu
 */

#include "sv650_reader.h"
#include "display.h"

extern bool bad_ecu;

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

   for (i = 1; i < 4; i++) {
       led.sendDigit(led_digits[i], error_table[idx].led[i-1]);
   } 
}

/*
 * Print water temp
 */
int 
print_led_temp() {
    unsigned int adc_value, temp;
    unsigned int rettemp = 0;
    int holding;
    char display[4];
#ifdef USE_CELCIUS
    float celcius;
#endif

    // Only decode the temp if we actually have data
    if (! bad_ecu) {
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
            temp = 500; // ludicrious value to warn of high temp
        } else if (adc_value >= 525) {
            // temp == LO (--- on stock display) 
            display[0] = get_display_char('L'); 
            display[1] = get_display_char('O'); 
            display[2] = 0;
            temp = 0;   // really cold!
        } else {
            // temp == XXXF
            temp = pgm_read_byte_near(temp_table + (adc_value - 42)) + 60;
#ifdef USE_CELCIUS
            // (F - 32) * 5/9 = C
            celcius = (float)(temp - 32) * ((float)5/9);
            temp = (unsigned int)celcius;
#endif
            dbg_serial_printf("temp is %u\n", temp);
            rettemp = temp;

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
    } else {
        // No temp available
        for (temp = 0; temp < 3; temp++) {
            display[temp] = get_display_char('-');
        }
        display[3] = get_display_char('F');
        rettemp = -1;
    }

    display_values(display[0], display[1], display[2], display[3], 0);
    return rettemp;
}

/*
 * Notify user of bad EFI condition
 */
void 
print_led_bad_ecu() {
    char display[4] = { 'B', 'A', 'D', 'E' };  // BADE
    dbg_serial_printf("print_led_bad_ecu\n");
    display_chars(display[0], display[1], display[2], display[3]);
}

/*
 * Print the current battery voltage
 */
void 
print_battery_voltage() {
    double voltage, batt_voltage;
    unsigned int batt_read, x;
    char display[4];
    char float_s[10];
    static unsigned long ms_last_volt = millis();
    unsigned long now = millis();
    static int first = 1;

    // Only update every BATT_VOLT_DELAY ms
    if (first || ((ms_last_volt + BATT_VOLT_DELAY) < now)) {
        ms_last_volt = now;
        first = 0;
    } else {
        return;
    }

    batt_read = analogRead(BATT_MON);

    dbg_serial_printf("analogRead battery: %u\n", batt_read);

    voltage = ((double)batt_read * AREAD_TO_VOLT) + ZENER_DROP_VOLTAGE;

#ifdef DEBUG
    ftoa(float_s, voltage, 2);
    serial_printf("voltage: %s\n", float_s);
#endif

    batt_voltage = (voltage * (R1 + R2)) / R2;

#ifdef DEBUG 
    ftoa(float_s, batt_voltage, 2);
    serial_printf("Battery voltage = %s\n", float_s);
#endif 

    x = batt_voltage * 10.0;
    batt_voltage = (float)x / 10.0;

    // alert user battery is low
    if (batt_voltage < BATT_VOLT_WARN) {
        low_batt = 1;
    }

    if (batt_voltage < 10.0) {
        display[0] = 0;
    } else {
        display[0] = (int)batt_voltage / 10;
    }
    display[1] = (int)batt_voltage % 10;
    display[2] = (batt_voltage - (int)batt_voltage) * 10;

    dbg_serial_printf("LED display voltage: %d%d%d%c\n", display[0],
            display[1], display[2], 'V');

    display[0] = get_display_hex(display[0]);
    display[1] = get_display_hex(display[1]);
    display[2] = get_display_hex(display[2]);
    display[3] = get_display_char('V');
    display_values(display[0], display[1], display[2], display[3], 0x02);
}
