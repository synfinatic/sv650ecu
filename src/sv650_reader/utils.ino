/* 
 * Copyright 2013-2014 (c) Aaron Turner
 * This code is released under the GPLv3 license.  Please see the LICENSE file 
 * for details.
 *
 * More information is available here: https://github.com/synfinatic/sv650ecu
 */

#include "utils.h"

extern byte sbytes[8];

/*
 * Returns GOOD_CSUM on valid, or what the checksum should of been
 * for the given array
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
        return GOOD_CSUM; // valid
    } else {
        return (unsigned int)check_csum; // invalid, return expected csum value
    }
}

/*
 * clears our serial ECU message buffer sbytes[]
 */
void 
clear_buf() {
    int j;
    for (j = 0; j < 8; j++) {
        sbytes[j] = 0;
    }
}

/*
 * printf to the HW serial port, useful for debugging.  Note 128char limit!
 */
void 
serial_printf(const char *fmt, ...) {
    char tmp[128]; // resulting string limited to 128 chars
    va_list args;
    va_start(args, fmt);
    vsnprintf(tmp, 128, fmt, args);
    va_end(args);
    Serial.print(tmp);
}

/*
 * printf to HW serial port, if we have debugging enabled
 */
void
dbg_serial_printf(const char *fmt, ...) {
#ifdef DEBUG
    char tmp[128]; // resulting string limited to 128 chars
    va_list args;
    va_start(args, fmt);
    vsnprintf(tmp, 128, fmt, args);
    va_end(args);
    Serial.print(tmp);
#else
    return;
#endif
}

/*
 * Float to ascii
 * Since the sprintf() of the Arduino doesn't support floating point
 * converstion, #include <stdlib.h> for itoa() and then use this function
 * to do the conversion manually
 */
char 
*ftoa(char *a, double f, int precision)
{
    long p[] = { 0,10,100,1000,10000,100000,1000000,10000000,100000000 };

    char *ret = a;
    long heiltal = (long)f;
    itoa(heiltal, a, 10);
    while (*a != '\0') a++;
    *a++ = '.';
    long desimal = abs((long)((f - heiltal) * p[precision]));
    itoa(desimal, a, 10);
    return ret;
}


/*
 * Takes a byte, deleta and pointer to the sbytes[] and updates sbytes
 * returns true when we have a valid message to decode
 * returns:
 * -1 = not enough data for full message
 *  0 = bad 8 byte message
 *  1 = valid good 8 byte message
 */
int
parse_ecu_byte(byte data, unsigned long delta)
{
    static int byte_idx = 0;          // message byte index
    unsigned int csum;
    int valid = -1;

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
#ifdef DEBUG_MESSAGE2
            serial_printf("%02x %02x %02x %02x %02x %02x %02x %02x [%lums] CRC ", 
                    sbytes[0], sbytes[1], sbytes[2], sbytes[3], 
                    sbytes[4], sbytes[5], sbytes[6], sbytes[7], delta);
#endif
            csum = check_csum();
            if (csum == GOOD_CSUM) {
#ifdef DEBUG_MESSAGE2
                Serial.print("OK\n");
#endif
                valid = true;
            } 
#ifdef DEBUG_MESSAGE2
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
    return valid;
}

/*
 * parse message and decode error meanings
 * If there is an error, is stores it in efi_error_code
 * If there are multiple errors, it will change efi_error_code every call
 * The TPS sensor value is stored in tps_adjust
 */
void
parse_message() {
    static int next_idx = 0;
    int i = 0;
    int last_efi_error_code = efi_error_code;

    tps_adjust = TPS_OK;
    efi_error_code = -1;


#ifdef DEBUG_MESSAGE
    serial_printf("Decoding Message: %02x %02x %02x %02x %02x %02x %02x %02x!\n", 
            sbytes[0], sbytes[1], sbytes[2], sbytes[3], 
            sbytes[4], sbytes[5], sbytes[6], sbytes[7]);
#endif

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
    } 

    /*
     * We loop through the error_table maximum of 2 times.  The goal is to
     * always return a valid error code if one exists in the sbytes[]
     * if after two trips (from where we left off) there are no more errors
     * then we'll return -1 indicating no errors
     */
    for (i = 0; i < 2; i++) {
        // Then check for errors thrown by the ECU
        while (error_table[next_idx].bindex != 0xff) {
            // matching error
            if ((sbytes[error_table[next_idx].bindex] & error_table[next_idx].mask) == 
                    error_table[next_idx].mask) {
#ifdef DECODE_ERRORS
                // print spacer line between each decode group
                if (next_idx == 0) {
                    serial_printf("************************\n");
                }
                serial_printf("[%d] %s\n", next_idx, error_table[next_idx].error);
#endif 
                efi_error_code = next_idx;
                next_idx++;
                break;
            }
            next_idx++;
        }
        if (efi_error_code > 0)
            break;
    }

    // Turn on EFI light if we have any errors other then dealer mode
    if (efi_error_code > 0) {
        efi_alarm = true;
    } else if (last_efi_error_code < 0) {
        /* 
         * turn off efi_alarm if we we have no error code 
         * AND the last call to parse_message() also had no 
         * error code.  This ensures we run through the entire
         * set of alarm codes before giving the all clear signal
         */
        efi_alarm = false;
    }
}

