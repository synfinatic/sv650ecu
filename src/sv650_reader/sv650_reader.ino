/* 
 * Copyright 2013 (c) Aaron Turner
 * This code is released under the GPLv3 license.  Please see the LICENSE file 
 * for details.
 *
 * More information is available here: http://synfin.net/sv650ecu
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

/*
 *  2c c8 80 08 40 00 00 44 [10ms] CRC OK
 *  2c c0 00 00 00 00 00 14 [9ms] CRC OK
 *  2c c8 80 00 00 00 00 8c [10ms] CRC OK 
 *
 *  Dealer Mode:
 *  2c da 80 08 40 00 00 32 [10ms] CRC OK
 *  DA = 11011010
 *  80 = 10000000
 *  08 = 00001000
 *  04 = 00000100
 *
 *  2c da 80 08 40 00 00 32 [10ms] CRC OK
 *
 */

/*
Message Decode: 

temp is 10 bits (0-1023), all of byte 0 and top two of byte 1
ecm_serial_byte[0] = temp >> 2
ecm_serial_byte[1].7 = temp.1
ecm_serial_byte[1].6 = temp.0

ecm_serial_byte[1].5 = C43 'Undefined for SV650
ecm_serial_byte[1].4 = dealer_mode
ecm_serial_byte[1].2 = Adj_TPS_1
ecm_serial_byte[1].1 = Adj_TPS_0
ecm_serial_byte[1].0 = C42_IG_Switch

ecm_serial_byte[2].7 = C41_FP_Relay
ecm_serial_byte[2].4 = C33_FI_2
ecm_serial_byte[2].3 = C32_FI_1
ecm_serial_byte[2].2 = C31_GPS
ecm_serial_byte[2].1 = C25_IG_Coil_2
ecm_serial_byte[2].0 = C24_IG_Coil_1

ecm_serial_byte[3].7 = C23_TOS
ecm_serial_byte[3].6 = C22_CAM 'Not used on SV650
ecm_serial_byte[3].5 = C21_IATS
ecm_serial_byte[3].4 = C15_ECTS
ecm_serial_byte[3].3 = C14_TPS
ecm_serial_byte[3].2 = C13_IAP
ecm_serial_byte[3].1 = C12_CKP
ecm_serial_byte[3].0 = C11 'Undefined for SV650


ecm_serial_byte[4].7 = C49_PAIR
ecm_serial_byte[4].6 = C29_STPS
ecm_serial_byte[4].5 = C28_STVA
ecm_serial_byte[4].3 = C44 'Undefined for SV650

ecm_serial_byte[5] = 0 // unused
ecm_serial_byte[6] = 0 // unused

ecm_serial_byte[7] = CRC

 */

#include <ctype.h>
#include <stdarg.h>
#include <ST6961.h>
#include <avr/pgmspace.h>

//#define DEBUG_MESSAGE         // decode each 8 byte message as hex
#define DECODE_ERRORS         // decode errors in messages
#define PRINT_DECODE 2000     // how often to decode error messages in ms
#define DEBUG                 // detailed debug
//#define DEBUG_TIMING
//#define DEBUG_TABLES        // Print our tables at startup via serial
#define ALWAYS_SHOW_ERRORS 1  // Show error codes even if not in dealer mode 1 or 0
// #define ENABLE_TEMP           // Enable decoding temperature
#define BLINK_MS 500          // how fast to blink status LED
#define ECU_SPEED 7800        // ECU serial speed

#define SERIAL_SPEED 9600     // Always 9600, really 12Mhz!
#define EFI_WARN 11           // EFI warning light
#define CS 0   // B0
#define CLK 1  // B1
#define MOSI 2 // B2
#define RX 7   // D2  UART pins!
#define TX 8   // D3 

// pins we don't use on the Teensy board
char unused_pins[] = { 
    3, 4, 5, 6, 9, 10, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24 
};

/*
 * Data struct to store all the ECU error code & meanings
 */
typedef struct _ECU_ERRORS
{
    byte bindex;              // byte index
    byte mask;                // mask bits
    byte led[3];              // LED code
    const char error[16];     // error string
} ECU_ERRORS;

/*
 * We comment out any error codes which aren't valid
 * or that we don't care about
 */
static const ECU_ERRORS tps_table[] = 
{
#define TPS_OK 0x40
    { 1, 0x04, { 0x01, 0, 0 }, "TPS Adj High"   },
    { 1, 0x02, { 0x08, 0, 0 }, "TPS Adj Low"    },
    { 1, 0x06, { 0x40, 0, 0 }, "TPS Adj Mid"    },
    { 0xff, 0xff, { 0, 0, 0 }, "" }, // last entry!
};
static const ECU_ERRORS error_table[] = 
{
#define DEALER_BINDEX 1
#define DEALER_MASK 0x10
    { 1, 0x10, { 0x39, 0x3f, 0x3f }, "C00 Dealer Mode"},
    { 1, 0x01, { 0x39, 0x66, 0x5b }, "C42 Ignit SW"   },
    { 2, 0x80, { 0x39, 0x66, 0x06 }, "C41 FP Relay"   },
    { 2, 0x10, { 0x39, 0x4f, 0x4f }, "C33 FI 2"       },
    { 2, 0x08, { 0x39, 0x4f, 0x5b }, "C32 FI 1"       },
    { 2, 0x04, { 0x39, 0x4f, 0x06 }, "C31 Gear Pos"   },
    { 2, 0x02, { 0x39, 0x5b, 0x6d }, "C25 IG Coil 2"  },
    { 2, 0x01, { 0x39, 0x5b, 0x66 }, "C24 IG Coil 1"  },
    { 3, 0x80, { 0x39, 0x5b, 0x4f }, "C23 Tip Over"   },
//    { 3, 0x40, { 0x39, 0x5b, 0x5b }, "C22 CAM Sensr"   },  
    { 3, 0x20, { 0x39, 0x5b, 0x06 }, "C21 Air Temp"   },
    { 3, 0x10, { 0x39, 0x06, 0x6d }, "C15 Eng Temp"   },
    { 3, 0x08, { 0x39, 0x06, 0x66 }, "C14 Pri TPS"    },
    { 3, 0x04, { 0x39, 0x06, 0x4f }, "C13 Air Press"  },
    { 3, 0x02, { 0x39, 0x06, 0x5b }, "C12 Crank Pos"  },
//    { 3, 0x01, { 0x39, 0x06, 0x06 }, "C11 ????"       },
    { 4, 0x80, { 0x39, 0x66, 0x67 }, "C49 Pair Valve" },
    { 4, 0x40, { 0x39, 0x5b, 0x67 }, "C29 Sec TPS"    },
    { 4, 0x20, { 0x39, 0x5b, 0x7f }, "C28 STVA Motor" },
//    { 4, 0x08, { 0x39, 0x66, 0x66 }, "C44 ????"       },
    { 0xff, 0xff, { 0x3f, 0x3f, 0x3f }, "000 No Error" }, // last entry!
};

// Starts at 42, each value is actually == value - 60
PROGMEM prog_uchar temp_table[] = 
{
    205, 203, 201, 199, 197, 196, 194, 192, 190, 188, 187, 185, 184, 183, 181,
    180, 179, 177, 176, 175, 173, 172, 171, 169, 168, 167, 165, 164, 163, 161,
    160, 159, 157, 156, 155, 153, 152, 151, 151, 150, 149, 148, 148, 147, 146,
    145, 145, 144, 143, 142, 142, 141, 140, 139, 139, 138, 137, 136, 136, 135,
    134, 133, 132, 132, 131, 130, 130, 129, 128, 127, 127, 126, 125, 124, 124,
    123, 122, 121, 121, 120, 119, 118, 118, 117, 116, 116, 115, 115, 114, 114,
    113, 113, 113, 112, 112, 111, 111, 110, 110, 109, 109, 109, 108, 107, 107,
    107, 106, 106, 106, 105, 105, 104, 104, 103, 103, 103, 102, 102, 101, 101,
    100, 100, 100, 99, 99, 98, 98, 97, 97, 96, 96, 96, 95, 95, 94, 94, 94, 93,
    93, 92, 92, 91, 91, 90, 90, 90, 89, 89, 88, 88, 87, 87, 87, 86, 86, 85, 85,
    84, 84, 83, 83, 83, 82, 82, 81, 81, 80, 80, 80, 79, 79, 79, 79, 78, 78, 78,
    78, 77, 77, 77, 76, 76, 76, 76, 75, 75, 75, 74, 74, 74, 74, 73, 73, 73, 73,
    72, 72, 72, 71, 71, 71, 71, 70, 70, 70, 69, 69, 69, 69, 68, 68, 68, 68, 67,
    67, 67, 66, 66, 66, 66, 65, 65, 65, 64, 64, 64, 64, 63, 63, 63, 63, 62, 62,
    62, 61, 61, 61, 61, 60, 60, 60, 60, 59, 59, 59, 58, 58, 58, 58, 57, 57, 57,
    56, 56, 56, 56, 55, 55, 55, 55, 54, 54, 54, 53, 53, 53, 53, 52, 52, 52, 51,
    51, 51, 51, 50, 50, 50, 50, 49, 49, 49, 48, 48, 48, 48, 47, 47, 47, 47, 46,
    46, 46, 45, 45, 45, 45, 44, 44, 44, 44, 43, 43, 43, 43, 43, 42, 42, 42, 42,
    42, 41, 41, 41, 41, 41, 40, 40, 40, 40, 40, 40, 39, 39, 39, 39, 39, 38, 38,
    38, 38, 38, 37, 37, 37, 37, 37, 36, 36, 36, 36, 36, 35, 35, 35, 35, 35, 34,
    34, 34, 34, 34, 33, 33, 33, 33, 33, 32, 32, 32, 32, 32, 31, 31, 31, 31, 31,
    31, 30, 30, 30, 30, 30, 29, 29, 29, 29, 29, 28, 28, 28, 28, 28, 27, 27, 27,
    27, 27, 26, 26, 26, 26, 26, 25, 25, 25, 25, 25, 24, 24, 24, 24, 24, 23, 23,
    23, 23, 23, 22, 22, 22, 22, 22, 22, 21, 21, 21, 21, 21, 20, 20, 20, 20, 20,
    19, 19, 19, 19, 19, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 16, 16, 16, 16,
    16, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 12, 12,
    12, 12, 12, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 8, 8, 8
};

PROGMEM prog_uchar display_table[] =
{
    0    , 0    , 0    , 0    , 0    , 0    , 0    , 0    , 
    0    , 0    , 0    , 0    , 0    , 0    , 0    , 0    , 
    0    , 0    , 0    , 0    , 0    , 0    , 0    , 0    , 
    0    , 0    , 0    , 0    , 0    , 0    , 0    , 0    , 
    0    , 0    , 0    , 0    , 0    , 0    , 0    , 0    , 
    0    , 0    , 0    , 0    , 0    , 0x40 , 0    , 0    , // -
    0x3f , 0x06 , 0x5b , 0x4f , 0x66 , 0x6d , 0x7d , 0x07 , // 0-7
    0x7f , 0x6f , 0    , 0    , 0    , 0    , 0    , 0    , // 8 9
    0    , 0x77 , 0x7c , 0x58 , 0x5e , 0x79 , 0x71 , 0x7d , // A-G
    0x76 , 0x06 , 0x07 , 0    , 0x34 , 0    , 0    , 0x3f , // H-O
    0x37 , 0    , 0    , 0x6d , 0x78 , 0xc3 , 0    , 0    , // P-W
    0    , 0    , 0    , 0x93 , 0    , 0xf0 , 0    , 0x80 , // [ ] _
    0    , 0x77 , 0x7c , 0x58 , 0x5e , 0x79 , 0x71 , 0x7d , // a-g
    0x76 , 0x06 , 0x07 , 0    , 0x34 , 0    , 0    , 0x3f , // h-o
    0x37 , 0    , 0    , 0x6d , 0x87 , 0xc3 , 0    , 0    , // p-w
    0    , 0    , 0    , 0    , 0    , 0    , 0    , 0
};


byte 
get_display_char(char val)
{
    return pgm_read_byte_near(display_table + int(val));
}

byte 
get_display_hex(int val)
{
    return pgm_read_byte_near(display_table + val + 0x30);
}

/* 
 * globals
 */
unsigned long ms_last;
int byte_idx = 0;
byte sbytes[8];
int efi_alarm = 0;  // true or false

HardwareSerial ecu = HardwareSerial();
ST6961 LED = ST6961(MOSI, CLK, CS);

const char DISPPOS[4] = { 0xc0, 0xc2, 0xc4, 0xc6 };

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

void 
display_values(char digit1, char digit2, char digit3, char digit4)
{

    digitalWrite(CS,LOW);
    shiftOut(MOSI, CLK, LSBFIRST, DISPPOS[0]);
    shiftOut(MOSI, CLK, LSBFIRST, digit1);
    shiftOut(MOSI, CLK, LSBFIRST, DISPPOS[1]);
    shiftOut(MOSI, CLK, LSBFIRST, digit2);
    shiftOut(MOSI, CLK, LSBFIRST, DISPPOS[2]);
    shiftOut(MOSI, CLK, LSBFIRST, digit3);
    shiftOut(MOSI, CLK, LSBFIRST, DISPPOS[3]);
    shiftOut(MOSI, CLK, LSBFIRST, digit4);
    digitalWrite(CS,HIGH);    
}

void 
display_char(int position, char digit)
{
    digitalWrite(CS,LOW);
    shiftOut(MOSI, CLK, LSBFIRST, DISPPOS[position]);
    shiftOut(MOSI, CLK, LSBFIRST, get_display_char(digit));
    digitalWrite(CS,HIGH);    
}

// clears our message buffer
void 
clear_buf()
{
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

/*
 * main setup 
 */
void 
setup() {
    unsigned i = 0;

    // initialize serial communications
    ecu.begin(ECU_SPEED);
    Serial.begin(SERIAL_SPEED);
    LED.initDisplay();

    // Put unused pins in output mode so they don't float
    for (i = 0; i < sizeof(unused_pins); i ++) {
       pinMode(unused_pins[i], OUTPUT);
    } 

    // count down through 0-9,a-f
    for(int i=16; i >= 0; i--) {
        LED.sendDigits(0,0,0,i,0);
        delay(100);
    } 
    LED.initDisplay();

    pinMode(EFI_WARN, OUTPUT);

    clear_buf();

    
#ifdef DEBUG_TABLES 
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
 * Print error code to LED display from the table
 */
void 
print_led_error(char tps_adjust, int idx) {
   int i = 0;
   byte led_digits[4] = { 0xc0, 0xc2, 0xc4, 0xc6 };

   LED.sendDigit(led_digits[0], tps_adjust);

    for (i = 1; i < 4; i++) {
        LED.sendDigit(led_digits[i], error_table[idx].led[i-1]);
    } 
}


#ifdef ENABLE_TEMP
/*
 * Print water temp
 */
void 
print_led_temp()
{
    unsigned int adc_value;
    unsigned int temp;
    int holding;
    char display[4];

    // temp is all of byte 0 and top two bits of byte 1
    adc_value = sbytes[0] << 2;
    adc_value += (sbytes[1] & 0xc0) >> 6;

    // covert temp into a value fareinheit
    // and then use LED.sendDigit()

    display[3] = get_display_char('F');
    if (adc_value <= 42) {
        // temp == HI 
        display[0] = get_display_char('H');
        display[1] = get_display_char('I');
        display[2] = 0;
    } else if (adc_value >= 525) {
        // temp == --- 
        display[0] = get_display_char('L'); 
        display[1] = get_display_char('O'); 
        display[2] = 0;
    } else {
        temp = pgm_read_byte_near(temp_table + (adc_value - 42)) + 60;
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
    display_values(display[0], display[1], display[2], display[3]);
}

void 
print_led_bad_temp()
{
    char display[4] = { 'B', 'A', 'D', 'T' };  // BADT
    serial_printf("print_led_bad_temp\n");
    display_chars(display[0], display[1], display[2], display[3]);
}
#endif

void 
print_led_bad_efi()
{
    char display[4] = { 'B', 'A', 'D', 'E' };  // BADE
    serial_printf("print_led_bad_efi\n");
    display_chars(display[0], display[1], display[2], display[3]);
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

/*
 * Main loop
 */
void 
loop()
{
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
