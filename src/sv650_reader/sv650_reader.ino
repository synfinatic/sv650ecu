/* 
 * Copyright 2013 (c) Aaron Turner
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
 * To support 7800 baud, you have to hack your SoftwareSerial.cpp file
 * and add this to the 16Mhz struct:
 *     { 7800,     138,       291,       291,      287,   },
 * numbers are based off the info in this thread:
 * http://arduino.cc/forum/index.php/topic,138497.0.html
 *
 * This code is written for a PJRC Teensy 2.0
 */


#include <ctype.h>
#include <stdarg.h>
#include <ST6961.h>
#include <avr/pgmspace.h>

#include "sv650_reader.h"
#include "display.h"

// pins we don't use on the Teensy board
char unused_pins[] = { 
    3, 4, 5, 6, 9, 10, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24 
};


/* 
 * globals
 */
unsigned long ms_last;
int byte_idx = 0;
byte sbytes[8];
int efi_alarm = 0;  // true or false

HardwareSerial ecu;
ST6961 led = ST6961(MOSI, CLK, CS);

void clear_buf();
void serial_printf(const char *fmt, ... );
void parse_message();
unsigned int check_csum();
void blink(unsigned long time);

/*
 * main setup 
 */
void 
setup() {
    unsigned i = 0;

    ecu = HardwareSerial();

    // initialize serial communications
    ecu.begin(ECU_SPEED);
    Serial.begin(SERIAL_SPEED);
    led.initDisplay();

    // Put unused pins in output mode so they don't float
    for (i = 0; i < sizeof(unused_pins); i++) {
       pinMode(unused_pins[i], OUTPUT);
    } 

    // count down through 0-9,a-f to look fancy and show it's working
    for (i = 16; i >= 0; i--) {
        led.sendDigits(0,0,0,i,0);
        delay(100);
    } 
    led.initDisplay();

    pinMode(EFI_WARN, OUTPUT);

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


    Serial.println("READY!");
    ms_last = millis();
}

/*
 * Main loop
 */
void 
loop() {
    unsigned long ms = 0;
    unsigned long delta;
    unsigned int csum;
    byte data;

    if (ecu.available() > 0) {
        // data is available to be read from the ECU
        ms = millis();
        data = (byte)ecu.read();
        delta = ms - ms_last;

        if (delta < 15) {
#ifdef DEBUG_TIMING
            serial_printf("Got %02x @ %lums (in time) for index: %d\n", data, delta, byte_idx);
#endif
            // data came in for our current message 
            if (byte_idx <= 6) {
                sbytes[byte_idx] = data;
                byte_idx++;
            } else if (byte_idx == 7) {
                // reached end of message!
                sbytes[byte_idx] = data;
#ifdef DEBUG_MESSAGE
                serial_printf("%02x %02x %02x %02x %02x %02x %02x %02x [%lums] CRC ", 
                        sbytes[0], sbytes[1], sbytes[2], sbytes[3], 
                        sbytes[4], sbytes[5], sbytes[6], sbytes[7], delta);
#endif
                csum = check_csum();
                if (csum == 0xffff) {
#ifdef DEBUG_MESSAGE
                    Serial.print("OK\n");
#endif
                    // only decode the bytes to an error message if checksum is OK
                    parse_message();

                    // print TEMP to LED if there is no alarm
#ifdef ENABLE_TEMP
                    if (! efi_alarm) {
                        print_led_temp();
                    }
#endif
                } 
#ifdef DEBUG_MESSAGE
                else {
                    serial_printf("ERROR: 0x%04x\n", csum);
                }
#endif
                byte_idx = 0;
            } else {
                serial_printf("ERROR: WTF???? byte_idx = %d\n", byte_idx);
                byte_idx = 0;
            }
        } else if (delta < 35) {
            // forced start of new message due to long wait

#ifdef DEBUG_MESSAGE
            if (byte_idx != 0) {
                serial_printf("%02x %02x %02x %02x %02x %02x %02x %02x BAD LEN: %d [%lums]\n", 
                        sbytes[0], sbytes[1], sbytes[2], sbytes[3], 
                        sbytes[4], sbytes[5], sbytes[6], sbytes[7], byte_idx+1, delta);
            }
#endif

            clear_buf();

            if (byte_idx != 0) {
                serial_printf("Got %02x @ %dms (late, forced restart) for index: %d\n", data, delta, -1);
            } else {
#ifdef DEBUG_TIMING
                serial_printf("Got %02x @ %dms (in time) for index 0\n", data, delta);
#endif
            }

            sbytes[0] = data;
            byte_idx = 1;
        } else {
            serial_printf("Crazy.. really long delay! %lums\n", delta);
        }
        ms_last = ms;
    } else {
        ms = millis() - ms_last;
        if (ms > 1000) {
            blink(ms);
        } else if (efi_alarm == 0) {
            digitalWrite(EFI_WARN, LOW);
        }
    }
}


/*
 * parse message and decode error meanings
 */
void
parse_message() {
    static unsigned int last_time = 0;
    static int next_idx = 0;
    int have_previous_error = 0;
    unsigned int delta = 0;
    unsigned int now;
    int i;
    int error = -1;
    char tps_adjust = TPS_OK;

    now = millis();
    // First time
    if (last_time == 0) {
        last_time = now;
    }

    delta = now - last_time;
    // only update display/serial every PRINT_DECODE ms
    if (delta >= PRINT_DECODE) {
        last_time = now;
        i = 0;

        // first check the TPS sensor 
        // we'll only print this if there is an error or we're in 
        // dealer mode!
        while (tps_table[i].bindex != 0xff) {
            if ((sbytes[tps_table[i].bindex] & tps_table[i].mask) == tps_table[i].mask) {
                tps_adjust = tps_table[i].led[0];
                break;
            }
            i++;
        }

        // see if we're at the end of the error_table[]
        if (error_table[next_idx].bindex == 0xff) {
            next_idx = 0;
        } else {
            have_previous_error = 1;
        }

        // Then check for errors thrown by the ECU
        while (error_table[next_idx].bindex != 0xff) {
            // matching error
            if ((sbytes[error_table[next_idx].bindex] & error_table[next_idx].mask) == error_table[next_idx].mask) {
#ifdef DECODE_ERRORS
                // print spacer line between each decode group
                if (next_idx == 0) {
                    serial_printf("************************\n");
                }
                serial_printf("[%d] %s\n", next_idx, error_table[next_idx].error);
#endif 
                error = next_idx;
                next_idx++;
                break;
            }
            next_idx++;
        }

        // Turn on EFI light if we have any errors other then dealer mode
        if (error > 0) {
            digitalWrite(EFI_WARN, HIGH);
            efi_alarm = 1;
#ifdef DECODE_ERRORS
            serial_printf("%02x %02x %02x %02x %02x %02x %02x %02x Has errors!\n", 
                    sbytes[0], sbytes[1], sbytes[2], sbytes[3], 
                    sbytes[4], sbytes[5], sbytes[6], sbytes[7]);
#endif
        } else if (have_previous_error == 0) {
            digitalWrite(EFI_WARN, LOW);
            efi_alarm = 0;
        }


        /* 
         * print errors on display if:
         * 1. dealer mode is enabled 
         * OR
         * 2. ALWAYS_SHOW_ERRORS == 1 is set and there is an error 
         */
        if ((efi_alarm && ALWAYS_SHOW_ERRORS) || ((sbytes[DEALER_BINDEX] & DEALER_MASK) == DEALER_MASK)) {
            print_led_error(tps_adjust, error);
        }
    }
}

/*
 * Returns 0xffff on valid, or what the checksum should of been
 * (>= 0) for the given array
 */
unsigned int 
check_csum() {
    byte check_sum = 0;

    // Calculate check sum byte  
    for (int x = 0; x <= 6; x++) {
        check_sum = check_sum + sbytes[x];
    }

    check_sum = 256 - check_sum;

    if (check_sum == sbytes[7]) {
        return 0xffff; // valid
    } else {
        return (unsigned int)check_csum; // invalid, return expected csum value
    }
}

/*
 *  Blink our status light
 */
void 
blink(unsigned long time) {
    static int hilo = 0;
    static unsigned long last = 0;
    unsigned int blink_time = BLINK_MS;
    unsigned long delta;

    // first time
    if (last == 0) {
        last = millis();
    }

    // blink faster if the EFI alarm is set
    if (efi_alarm == 1) {
        blink_time /= 4;
    }

    delta = millis() - last;
    if (delta >= blink_time) {
        hilo = ! hilo;
        digitalWrite(EFI_WARN, hilo);
        last = millis();
        if (hilo == HIGH) {
            serial_printf("Missing data for %lu seconds\n", (unsigned long)(time / 1000));
        }
    }
}


// clears our message buffer
void 
clear_buf() {
    int j;
    for (j = 0; j < 8; j++) {
        sbytes[j] = 0;
    }
}

// printf to the HW serial port.  Note 128char limit!
void 
serial_printf(const char *fmt, ... ) {
    char tmp[128]; // resulting string limited to 128 chars
    va_list args;
    va_start(args, fmt);
    vsnprintf(tmp, 128, fmt, args);
    va_end(args);
    Serial.print(tmp);
}

