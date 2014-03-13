/* 
 * Copyright 2013 Aaron Turner
 *
 * Arduino Uno based SV650 ECU TACH line emulator.  I wrote this to 
 * make it easier to test my sv650 decoder code.  
 *
 */

// #define USE_TEENSY  // Define if using Teensy instead of Arduino Uno

#ifndef USE_TEENSY
#include <SWSerial7800.h>
#endif

#include <ST6961.h>
#define CS 10
#define MOSI 11
#define CLK 13

#define RX 7
#define TX 8                // ECU pin  
#define BUTTON 2
#define INTERRUPT 0         // INT for pin 2

#define ECU_SPEED 7800
#define SERIAL_SPEED 9600
#define BLINK_LED 13 
#define BLINK_MS 500
#define DEBOUNCE_MS 1000

volatile bool change_ecu_code = 0;


#ifndef USE_TEENSY
// init SWSerial7800 for ECU communications.  Need to keep ecu global
SWSerial7800 ecu(RX, TX);
#else
HardwareSerial ecu = HardwareSerial();
#endif
ST6961 LED(MOSI, CLK, CS);


/*
 *  Error codes to rotate through
 */
typedef struct _ECU_CODES {
    byte bytes[8];
} ECU_CODES;

static ECU_CODES ecu_codes_table[] =
{
    { 0x5a , 0x0  , 0x0  , 0    , 0    , 0    , 0    , 0 }    ,  // 100F
    { 0x1c , 0xc0 , 0x0  , 0    , 0    , 0    , 0    , 0 }    ,  // 184F
    { 0x10 , 0xc0 , 0x0  , 0    , 0    , 0    , 0    , 0 }    ,  // 227F
    { 0x2c , 0xc8 , 0x0  , 0    , 0x40 , 0    , 0    , 0 }    ,  // 227F, C29
    { 0x0a , 0x08 , 0x80 , 0    , 0    , 0    , 0    , 0 }    ,  // HI F, C41
    { 0x2c , 0xc8 , 0x80 , 0    , 0xa0 , 0    , 0    , 0 }    ,  // 153F, C28, C41, C49
    { 0xff , 0xff , 0xff , 0xff , 0xff , 0xff , 0xff , 0xff }
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

void
button_interrupt(void)
{
    static unsigned long last_time = 0;
    unsigned long now;

    now = millis();
    if ((last_time + DEBOUNCE_MS) < now) {
        change_ecu_code = 1;
        last_time = now;
    }
}



/*
 *  Initialize everything
 */
void 
setup() {
    unsigned int i = 0;
    ecu.begin(ECU_SPEED);
    Serial.begin(SERIAL_SPEED);
    pinMode(BUTTON, INPUT_PULLUP);
    attachInterrupt(INTERRUPT, button_interrupt, FALLING);


    // calc checksums for all the messages
    while (ecu_codes_table[i].bytes[0] != 0xff) {
        ecu_codes_table[i].bytes[7] = gen_csum(i);
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

    if (change_ecu_code) {
        serial_printf("%s", "button pressed!\n");
        change_ecu_code = 0;
        if (ecu_codes_table[ecu_code+1].bytes[0] == 0xff) {
            ecu_code = 0;
        } else {
            ecu_code ++;
        }
        serial_printf("Sending: %02x %02x %02x %02x %02x %02x %02x %02x\n", 
            ecu_codes_table[ecu_code].bytes[0], 
            ecu_codes_table[ecu_code].bytes[1], 
            ecu_codes_table[ecu_code].bytes[2], 
            ecu_codes_table[ecu_code].bytes[3], 
            ecu_codes_table[ecu_code].bytes[4], 
            ecu_codes_table[ecu_code].bytes[5], 
            ecu_codes_table[ecu_code].bytes[6], 
            ecu_codes_table[ecu_code].bytes[7]);
    }

    // Write data bytes to serial
    for (x = 0; x <= 7; x++) {
        ecu.write(ecu_codes_table[ecu_code].bytes[x]);
        delay(10);
    }

    // Intermessage delay
    delay(20);  // only use 20ms because we already slept 10ms earlier
    blink();
}
