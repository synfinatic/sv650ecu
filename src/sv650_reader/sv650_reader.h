/* 
 * Copyright 2013-2014 (c) Aaron Turner
 * This code is released under the GPLv3 license.  Please see the LICENSE file 
 * for details.
 *
 * More information is available here: https://github.com/synfinatic/sv650ecu
 */

#ifndef SV650_READER_H
#define SV650_READER_H

/* 
 * Feel free to change these values based on how you want 
 * the ECU Decoder to behave
 */
//#define ENABLE_TEMP         // Enable decoding temperature
#define USE_CELCIUS           // Display in C, not F

#define DECODE_ERRORS         // decode ECU codes to serial
#define PRINT_DECODE 2000     // how often to decode error messages in ms
#define ALWAYS_SHOW_ERRORS 1  /* Show error codes even if not in dealer mode:
                               * 1 = always show errors 
                               * 0 = only in dealer mode
                               */

#define BLINK_MS 500          // how fast to blink EFI Warning light on no-data error

#define ENABLE_BATT_MONITOR   // Enable monitoring battery voltage
#define BATT_VOLT_WARN 13.3   // Voltage to start warning at (float)

/*
 * Debugging options.
 */
//#define DEBUG_MESSAGE       // decode each 8 byte message as hex
#define DEBUG               // detailed debug
//#define DEBUG_TIMING
//#define DEBUG_TABLES        // Print our tables at startup via serial

/****************************************************************************
 * Don't anything below this point unless you really know what you are doing!
 ****************************************************************************/
#define ECU_SPEED 7800        // ECU serial speed

#define SERIAL_SPEED 9600     // Always 9600, really 12Mhz!
#define EFI_WARN 11           // EFI warning light
#define CS 0   // B0
#define CLK 1  // B1
#define MOSI 2 // B2
#define RX 7   // D2  UART pins!
#define TX 8   // D3 

// Votage monitoring values
#define BATT_MON A0
#define R1 32000.0   // R5 + R8
#define R2 10000.0   // R6
#define AREAD_TO_VOLT 0.0049

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
 * Throttle position adjustment table
 */
static const ECU_ERRORS tps_table[] = 
{
#define TPS_OK 0x40
    { 1, 0x04, { 0x01, 0, 0 }, "TPS Adj High"   },
    { 1, 0x02, { 0x08, 0, 0 }, "TPS Adj Low"    },
    { 1, 0x06, { 0x40, 0, 0 }, "TPS Adj Mid"    },
    { 0xff, 0xff, { 0, 0, 0 }, "" }, // last entry!
};

/*
 * Error table of all known SV650 error codes.
 * The commented out codes I believe are correct 
 * for a DL1000 but are untested
 */
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
//    { 3, 0x40, { 0x39, 0x5b, 0x5b }, "C22 Atmosphere" },  
    { 3, 0x20, { 0x39, 0x5b, 0x06 }, "C21 Air Temp"   },
    { 3, 0x10, { 0x39, 0x06, 0x6d }, "C15 Eng Temp"   },
    { 3, 0x08, { 0x39, 0x06, 0x66 }, "C14 Pri TPS"    },
    { 3, 0x04, { 0x39, 0x06, 0x4f }, "C13 Air Press"  },
    { 3, 0x02, { 0x39, 0x06, 0x5b }, "C12 Crank Pos"  },
//    { 3, 0x01, { 0x39, 0x06, 0x06 }, "C11 CAM Shaft"  },
    { 4, 0x80, { 0x39, 0x66, 0x67 }, "C49 Pair Valve" },
    { 4, 0x40, { 0x39, 0x5b, 0x67 }, "C29 Sec TPS"    },
    { 4, 0x20, { 0x39, 0x5b, 0x7f }, "C28 STVA Motor" },
//    { 4, 0x08, { 0x39, 0x66, 0x66 }, "C44 Heated O2"  },
    { 0xff, 0xff, { 0x3f, 0x3f, 0x3f }, "000 No Error" }, // last entry!
};

/*
 * The ECU has an ADC to read the water temp sensor and sends values 
 * which we have to map to degrees F.  
 * Values 0-42 are "HI"
 * Values 43-542 are mapped to degrees F below
 * Values > 543 are --- (too low)
 *
 * The map values below are actually 60F below the actual temp because
 * I wanted to store all possible values (265-68F) in a single byte.
 */

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

/*
 * Maps ASCII characters to something that looks like that character 
 * on the 7 segment display.  Not all characters are represented and 
 * some chars overlap (especially upper & lower case).
 */
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
    0x76 , 0x06 , 0x07 , 0    , 0x38 , 0    , 0    , 0x3f , // H-O
    0x37 , 0    , 0    , 0x6d , 0x78 , 0xc3 , 0x1c , 0    , // P-W
    0    , 0    , 0    , 0x93 , 0    , 0xf0 , 0    , 0x80 , // [ ] _
    0    , 0x77 , 0x7c , 0x58 , 0x5e , 0x79 , 0x71 , 0x7d , // a-g
    0x76 , 0x06 , 0x07 , 0    , 0x38 , 0    , 0    , 0x3f , // h-o
    0x37 , 0    , 0    , 0x6d , 0x87 , 0xc3 , 0x1c,  0    , // p-w
    0    , 0    , 0    , 0    , 0    , 0    , 0    , 0
};

#endif
