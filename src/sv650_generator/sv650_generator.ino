/* 
 * Copyright 2013 Aaron Turner
 *
 * Arduino based SV650 ECU TACH line emulator.  I wrote this to 
 * make it easier to test my sv650 decoder code.  
 *
 * To support 7800 baud, you have to hack your SoftwareSerial.cpp file
 * and add this to the 16Mhz struct:
 *     { 7800,     138,       291,       291,      287,   },
 */
#include <SoftwareSerial.h>
#include <ST6961.h>
#define CS 10
#define MOSI 11
#define CLK 13

#define RX 7
#define TX 8                // ECU pin  
#define BUTTON 4

#define ECU_SPEED 7800
#define SERIAL_SPEED 115200
#define BLINK_LED 13 
#define BLINK_MS 500

int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// the following variables are long's because the time, measured in miliseconds,
// // will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers


// init SoftwareSerial for ECU communications.  Need to keep ecu global
SoftwareSerial ecu(RX, TX);
ST6961 LED(MOSI, CLK, CS);


/*
 *  Error codes to rotate through
 */
typedef struct _ECU_CODES {
    byte bytes[8];
} ECU_CODES;

static ECU_CODES ecu_codes_table[] =
{
    { 0x5a , 0x10 , 0x0  , 0    , 0    , 0    , 0    , 0 }    , 
    { 0x1c , 0xc0 , 0x0  , 0    , 0    , 0    , 0    , 0 }    , 
    { 0x10 , 0xc0 , 0x0  , 0    , 0    , 0    , 0    , 0 }    , 
    { 0x2c , 0xc8 , 0x0  , 0    , 0x40 , 0    , 0    , 0 }    , 
    { 0x2c , 0xc8 , 0x80 , 0    , 0    , 0    , 0    , 0 }    , 
    { 0x2c , 0xc8 , 0x80 , 0    , 0xa0 , 0    , 0    , 0 }    , 
    { 0xff , 0xff , 0xff , 0xff , 0xff , 0xff , 0xff , 0xff }
};

// How long to show each error code
static unsigned int code_times[] = 
{
    4000, 3000, 3000, 6000, 6000, 10000
};


// printf to the HW serial port
void 
serial_printf(char *fmt, ... ) {
    char tmp[128]; // resulting string limited to 128 chars
    va_list args;
    va_start (args, fmt );
    vsnprintf(tmp, 128, fmt, args);
    va_end (args);
    Serial.print(tmp);
}

/*
 * Calculate the checksum
 */
byte
gen_csum(unsigned int code) {
    byte check_sum = 0;

    // Calculate check sum byte  
    for (int x = 0; x <= 6; x++) {
        check_sum = check_sum + ecu_codes_table[code].bytes[x];
    }

    check_sum = 256 - check_sum;

    return check_sum;
}

/*
 *  Initialize everything
 */
void 
setup() {
    unsigned int i = 0;
    ecu.begin(ECU_SPEED);
    Serial.begin(SERIAL_SPEED);
    pinMode(BUTTON, INPUT);

    // calc checksums for all the messages
    while (ecu_codes_table[i].bytes[0] != 0xff) {
        serial_printf("calculating checksum for %u\n", i);
        ecu_codes_table[i].bytes[7] = gen_csum(i);
        serial_printf("OK\n");
        i++;
    }
    serial_printf("Sending: %02x %02x %02x %02x %02x %02x %02x %02x\n", 
        ecu_codes_table[0].bytes[0], 
        ecu_codes_table[0].bytes[1], 
        ecu_codes_table[0].bytes[2], 
        ecu_codes_table[0].bytes[3], 
        ecu_codes_table[0].bytes[4], 
        ecu_codes_table[0].bytes[5], 
        ecu_codes_table[0].bytes[6], 
        ecu_codes_table[0].bytes[7]);
}
/*
 *  Blink our status light
 */
void 
blink() {
    static int hilo = 0;
    static unsigned long last = millis();

    if (millis() - last >= BLINK_MS) {
        hilo = ! hilo;
        digitalWrite(BLINK_LED, hilo);
        last = millis();
    }
}

void 
loop() {
    int x;
    static unsigned int ecu_code = 0;
    static unsigned long message_ms = millis();
    unsigned long now;

    // Write data bytes to serial
    for (x = 0; x <= 7; x++) {
        ecu.write(ecu_codes_table[ecu_code].bytes[x]);
        delay(10);
    }

    // Intermessage delay
    delay(20);  // only use 20ms because we already slept 10ms earlier
    blink();


    // Do we go to the next message?
    now = millis();
    if (now >= message_ms + code_times[ecu_code]) {
        message_ms = now;
        // go to next code
        if (ecu_codes_table[ecu_code+1].bytes[0] == 0xff) {
            ecu_code = 0;
        } else {
            ecu_code ++;
        }
        serial_printf("Sending for %ums: %02x %02x %02x %02x %02x %02x %02x %02x\n", 
            code_times[ecu_code],
            ecu_codes_table[ecu_code].bytes[0], 
            ecu_codes_table[ecu_code].bytes[1], 
            ecu_codes_table[ecu_code].bytes[2], 
            ecu_codes_table[ecu_code].bytes[3], 
            ecu_codes_table[ecu_code].bytes[4], 
            ecu_codes_table[ecu_code].bytes[5], 
            ecu_codes_table[ecu_code].bytes[6], 
            ecu_codes_table[ecu_code].bytes[7]);
    }
}
