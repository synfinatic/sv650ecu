/* 
 * Copyright 2014 (c) Aaron Turner
 * This code is released under the GPLv3 license.  Please see the LICENSE file 
 * for details.
 *
 * More information is available here: https://github.com/synfinatic/sv650ecu
 *
 * Suzuki SV650 2nd gen (03-06) ECU -> Dash protocol decoder
 * Black/Green wire from ECU to Dash.  Simple serial protocol
 * running at 7800 BAUD.  Each message is 8 bytes.  10 messages/sec
 * 10ms gap between bytes, 30ms gap between messages
 *
 * This code is written for a PJRC Teensy 2.0
 */


#include <ctype.h>
#include <stdarg.h>
#include <ST6961.h>
#include <MsTimer2.h>
#include <avr/pgmspace.h>

#include "sv650_reader.h"
#include "display.h"
#include "print.h"
#include "utils.h"

// pre-declare our functions
void mode_button_interrupt();
void print_led_display();
void select_next_mode();
void flash_update();
void flash_fast_update();
void no_flash_update();

/****************************************************************************
 * globals
 ****************************************************************************/

// pins we don't use on the Teensy board
static char used_pins[] = { 
    EFI_WARN, CS, CLK, MOSI, MODE_PIN, RX, TX, BATT_MON_DIGITAL, 
    FUEL_DIGITAL, FUEL2_DIGITAL
};

// Order of 4-LED display modes
static const mode mode_list[] = {
#ifdef ENABLE_TEMP
    MODE_TEMP,
#endif
#ifdef ENABLE_BATT
    MODE_BATTERY,
#endif
    MODE_EFI_ERROR,
    MODE_BAD_ECU,
    MODE_LAST
};

byte sbytes[8];              // storage for our messages from the ECU
unsigned int last_efi_light = 0;
bool efi_alarm = false;       // true or false if we've seen an error code
bool bad_ecu   = false;         // true or false if we have EFI communication issues
bool low_batt  = false;        // true or false for low voltage
bool bad_temp  = false;        // true or flase for water temp too high
#ifdef ENABLE_TEMP
mode current_mode = MODE_TEMP;
#elif defined ENABLE_TEMP
mode current_mode = MODE_BATTERY;  // current 4-LED display mode
#else
mode current_mode = MODE_CLEAR;
#endif
volatile bool mode_button_pressed = false;
char tps_adjust;
int efi_error_code = -1;
bool valid_message = false;

// Init our hardware serial & LCD display
HardwareSerial ecu = HardwareSerial();
ST6961 led = ST6961(MOSI, CLK, CS);

/****************************************************************************
 * main setup 
 ****************************************************************************/
void 
setup() {
    int i;
    unsigned int j;
    bool pin_is_used;

    // Put unused pins in output mode so they don't float
    for (i = 0; i <= 24;  i++) {
        pin_is_used = false;
        for (j = 0; j < sizeof(used_pins); j++) {
            if (i == used_pins[j]) {
                pin_is_used = true;
                break;
            }
        }
        if (! pin_is_used) {
#ifdef DEBUG_TABLES
            serial_printf("Marking pin #%d as OUTPUT\n", i);
#endif
            pinMode(i, OUTPUT);
        }
    } 

    // initialize serial communication to the ECU
    Serial.begin(SERIAL_SPEED);
    Serial.println("begin setup()");
    ecu.begin(ECU_SPEED);

    // Make sure BATT_MON pin is INPUT or we'll fry the board
    pinMode(BATT_MON, INPUT);
    
    // attach interrupt handler for the mode button
    pinMode(MODE_PIN, INPUT_PULLUP);
    attachInterrupt(MODE_INT, mode_button_interrupt, FALLING);

    // we control the EFI warning LED
    pinMode(EFI_WARN, OUTPUT);

    // initalize the LED display
    led.initDisplay();

    // clear display
    display_chars(' ', ' ', ' ', ' ');

    // clear sbytes[]
    clear_buf();
   
#ifdef DEBUG_TABLES 
    /*
     *  Prints out all our static tables to serial for debugging/info purposes
     */ 
    i = 0;
    serial_printf("our tps_table[] size is %u bytes\n", sizeof(tps_table));
    while (tps_table[i].bindex != 0xff) {
        serial_printf("tps_table[%d].bindex = '%02x'\n", i, tps_table[i].bindex);
        serial_printf("tps_table[%d].mask   = '%02x'\n", i, tps_table[i].mask);
        serial_printf("tps_table[%d].error  = '%s'\n", i, tps_table[i].error);
        i++;
    }

    serial_printf("our error_table[] size is %u bytes\n", sizeof(error_table));
    while (error_table[i].bindex != 0xff) {
        serial_printf("error_table[%d].bindex = '%02x'\n", i, error_table[i].bindex);
        serial_printf("error_table[%d].mask   = '%02x'\n", i, error_table[i].mask);
        serial_printf("error_table[%d].error  = '%s'\n", i, error_table[i].error);
        i++;
    }

    serial_printf("%s", "our display_table[]:\n");
    for (i = 0; i <= 127; i++) {
        serial_printf("%c = %c\n", i, pgm_read_byte_near(display_table +  i));
    }
#endif 

    digitalWrite(EFI_WARN, LOW);
    MsTimer2::start();
    MsTimer2::set(TIMER_MS, no_flash_update);

    Serial.println("READY!");
}

/****************************************************************************
 * main loop 
 ****************************************************************************/
void 
loop() {
    static unsigned long ms_last = 0;     // last time we saw a message
    unsigned long delta, now, parse_delta;
    static bool efi_alarm_light_on = false;
    static bool temp_alarm_light_on = false;
    static unsigned long last_parse = 0;
    byte data;
    int count, i, valid;

    now = millis();
    delta = now - ms_last;
    parse_delta = now - last_parse;


    count = ecu.available();
    for (i = 0; i < count; i++) {
        // data is available to be read from the ECU
        data = (byte)ecu.read();
        ms_last = now;

        /*
         * Figure out if we got a complete & valid message
         */
        valid = parse_ecu_byte(data, delta);
        if (valid == 0) {
            valid_message = false;
        } else if (valid == 1) {
            valid_message = true;
        }

        // only parse valid messages every DECODE_MS 
        if ((valid != -1) && valid_message && (parse_delta >= DECODE_MS)) {
            parse_message();
            last_parse = now;
            print_led_display();
        }

        /*
         * Figure out which *_update() function to call via MsTimer2
         */
        if (bad_ecu) {
            // Turn off the EFI light if necessary
            dbg_serial_printf("ecu comms restored! Turning off light.\n");
            bad_ecu = false;
            MsTimer2::set(TIMER_MS, no_flash_update);
            digitalWrite(EFI_WARN, LOW);
        } else if (bad_temp && ! temp_alarm_light_on) {
            // Temperature is too high, flash the light fast
            dbg_serial_printf("bad temp.  Flashing fast.\n");
            temp_alarm_light_on = true;
            MsTimer2::set(TIMER_MS, flash_fast_update);
        } else if (efi_alarm && ! efi_alarm_light_on) {
            // We have an EFI error, so solid EFI light
            dbg_serial_printf("efi alarm, solid light.\n");
            efi_alarm_light_on = true;
            MsTimer2::set(TIMER_MS, no_flash_update);
            digitalWrite(EFI_WARN, HIGH);
        }  else if (! efi_alarm && efi_alarm_light_on) {
            // No longer have EFI error, so turn off the EFI light
            dbg_serial_printf("efi alarms disabled.  Turning off light.\n");
            efi_alarm_light_on = false;
            MsTimer2::set(TIMER_MS, no_flash_update);
            digitalWrite(EFI_WARN, LOW);
        } else if (! bad_temp && temp_alarm_light_on) {
            // Temperature is low again, stop flashing
            dbg_serial_printf("temp is now ok.  Turning off light.\n");
            temp_alarm_light_on = false;
            MsTimer2::set(TIMER_MS, no_flash_update);
            digitalWrite(EFI_WARN, LOW);
        }
    } 
    if (! count) {
        // Check to see if ECU is not available
        if ((delta > NO_ECU_WARN_DELAY) && ! bad_ecu) {
            dbg_serial_printf("bad ecu comms.  Flashing slowly.\n");
            bad_ecu = true;
            MsTimer2::set(TIMER_MS, flash_update);

            // Clear any EFI alarm errors
            efi_alarm = false;
        }
    }

    // always update the display if user pressed the mode button
    if (mode_button_pressed) {
        print_led_display();
    }
}

/*
 * Just update the LED display
 */
void
no_flash_update() {
    if (! valid_message) {
        print_led_display();
    }
}

/*
 * Flash the EFI error really fast for bad temperature
 * and update the LED display
 */
void 
flash_fast_update() {
    static boolean hilo = 0;
    static unsigned long blink_time = 0;
    unsigned long now;

    now = millis();
    if (blink_time == 0) {
        blink_time = now + (BLINK_MS/4);
    }

    if (now > blink_time) {
        blink_time = now + (BLINK_MS/4);
        digitalWrite(EFI_WARN, hilo);
        hilo = !hilo;
    }

    if (! valid_message) {
        print_led_display();
    }

}

/*
 * Flash EFI error light & update the LED display
 */
void 
flash_update() {
    static boolean hilo = 0;
    static unsigned long blink_time = 0;
    unsigned long now;

    now = millis();
    if (blink_time == 0) {
        blink_time = now + BLINK_MS;
    }

    if (now > blink_time) {
        blink_time = now + BLINK_MS;
        digitalWrite(EFI_WARN, hilo);
        hilo = !hilo;
    }
    
    if (! valid_message) {
        print_led_display();
    }
}


/*
 * Figures out what mode the display should be after each button press
 */
void
mode_button_interrupt()
{
    static unsigned long last_time = 0;
    unsigned long now;

    now = millis();
    if ((last_time + DEBOUNCE_MS) < now) {
        mode_button_pressed = true;
        last_time = now;
    }
}

/*
 * Based on our current display mode, figure out what the next
 * valid mode is
 */
void
select_next_mode()
{
    bool again = true;
    int max_times = 0;
    static unsigned int mode_index = MODE_LAST;

    dbg_serial_printf("select_next_mode(): Current mode: %u\t\t", current_mode);

    max_times = MODE_CLEAR * 2;

    while (again && max_times) {
        if (mode_index >= MODE_LAST) {
            mode_index = 0;
        } else {
            mode_index += 1;
        }
        current_mode = mode_list[mode_index];

        // See if we need to look for the next mode
        again = false;
        if (((current_mode == MODE_BAD_ECU) && ! bad_ecu) ||
            ((current_mode == MODE_EFI_ERROR) && ! efi_alarm)) {
            again = true;
        }
        max_times -= 1;
    }
    if (again && ! max_times) {
        current_mode = MODE_CLEAR;
    }
    dbg_serial_printf("New mode: %u\n", current_mode);
}


void
clear_efi_light(unsigned int code)
{
    if ((last_efi_light & code) == code) {
        last_efi_light -= code;
    }
}

void
set_efi_light(int code)
{
    if ((last_efi_light & code) == 0) {
        last_efi_light += code;
    }
}

/*
 *  Figures out what to display on the 4-LED
 */
void
print_led_display()
{
    static mode last_mode = MODE_LAST;

    /*
     * If a new bad condition (EFI error, no EFI communication or low battery)
     * then switch to that display mode.  Order of the if/else sets the priority.
     */
    if (mode_button_pressed) {
        mode_button_pressed = false;
        dbg_serial_printf("Mode button pressed\n");
        select_next_mode();
    } else if (((last_efi_light & EFI_LIGHT_BAD_ECU) == 0) && bad_ecu) {
        // bad efi enabled
        current_mode = MODE_BAD_ECU;
        set_efi_light(EFI_LIGHT_BAD_ECU);
    } else if (((last_efi_light & EFI_LIGHT_ALARM) == 0) && efi_alarm) {
        // efi error enabled
        current_mode = MODE_EFI_ERROR;
        set_efi_light(EFI_LIGHT_ALARM);
#ifdef ENABLE_BATT
    } else if (((last_efi_light & EFI_LIGHT_LOW_BATT) == 0) && low_batt) {
        // low battery enabled
        current_mode = MODE_BATTERY;
        set_efi_light(EFI_LIGHT_LOW_BATT);
#endif
    } else if (((last_efi_light & EFI_LIGHT_BAD_ECU) == EFI_LIGHT_BAD_ECU) && ! bad_ecu) {
        // clear bad efi alarm
        clear_efi_light(EFI_LIGHT_BAD_ECU);
        if (current_mode == MODE_BAD_ECU) {
            current_mode = MODE_LAST;
            select_next_mode();
        }
    } else if (((last_efi_light & EFI_LIGHT_ALARM) == EFI_LIGHT_ALARM) && ! efi_alarm) {
        // clear efi alarm
        clear_efi_light(EFI_LIGHT_ALARM);
        if (current_mode == MODE_EFI_ERROR) {
            current_mode = MODE_LAST;
            select_next_mode();
        }
#ifdef ENABLE_BATT
    } else if (((last_efi_light & EFI_LIGHT_LOW_BATT) == EFI_LIGHT_LOW_BATT) && ! low_batt) {
        // clear low battery warning
        clear_efi_light(EFI_LIGHT_LOW_BATT);
        if (current_mode == MODE_BATTERY) {
            current_mode = MODE_LAST;
            select_next_mode();
        }
#endif
    }

#ifdef DEBUG
    if (current_mode != last_mode) {
        serial_printf("display mode changing! %d -> %d\n", last_mode, current_mode);
    }
#endif

    switch (current_mode) {
        case MODE_BAD_ECU:
            print_led_bad_ecu();
            break;
        case MODE_EFI_ERROR:
            dbg_serial_printf("Updating EFI error message 0x%04x\n", efi_error_code);
            print_led_error(tps_adjust, efi_error_code, efi_alarm);
            break;
#ifdef ENABLE_BATT
        case MODE_BATTERY:
            print_battery_voltage();
            break;
#endif
#ifdef ENABLE_TEMP
        case MODE_TEMP:
            if (print_led_temp() >= TEMP_WARN) {
                bad_temp = true;
            }
            break;
#endif
        default:
            serial_printf("Invalid current_mode %u.  Won't print anything\n",
                    (unsigned int)current_mode);
            select_next_mode();
    }
    last_mode = current_mode;
}
