/* 
 * Copyright 2013 Aaron Turner
 *
 * Wrote this code to make it easy to built the temperature ADC map
 * since the SV650 ECU has the ADC and it's up to the dash to convert
 * that into degrees F/C.  I used this with the OEM dash to figure out
 * what ADC values mapped to what temperatures.
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
#define UP_BUTTON 4
#define DOWN_BUTTON 3

#define ECU_SPEED 7800
#define BLINK_LED 13 
#define BLINK_MS 500
int sbytes[8];     // 7 bytes for serial data

// init SWSerial7800 for ECU communications.  Need to keep ecu global
#ifndef USE_TEENSY
SWSerial7800 ecu(RX, TX);
#else
HardwareSerial ecu = HardwareSerial()
#endif
ST6961 LED(MOSI, CLK, CS);


void 
clear_buf()
{
    int j;
    for (j = 0; j < 7; j++) {
        sbytes[j] = 0;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.print("Starting\n");
    LED.initDisplay();
    ecu.begin(ECU_SPEED);
    clear_buf();
    calc_csum();
    pinMode(UP_BUTTON, INPUT);
    pinMode(DOWN_BUTTON, INPUT);
}

/*
 * calculates the checksum of sbytes[]
 */
void  
calc_csum() {
    byte check_sum = 0;

    // Calculate check sum byte  
    for (int x = 0; x <= 6; x++) {
        check_sum = check_sum + sbytes[x];
    }

    sbytes[7] = 256 - check_sum;
}

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

unsigned int 
increment_temp() 
{
    unsigned int temp = 0;
    temp = sbytes[0] << 2;
    temp += (sbytes[1] & 0xc0) >> 6;

    temp += 1;
    sbytes[0] = temp >> 2;
    sbytes[1] = temp << 6;

    serial_printf("temp = %04u\n", temp);
    calc_csum();
    return temp;
}

unsigned int 
decrement_temp() 
{
    unsigned int temp = 0;
    temp = sbytes[0] << 2;
    temp += (sbytes[1] & 0xc0) >> 6;

    temp -= 1;

    sbytes[0] = temp >> 2;
    sbytes[1] = temp << 6;

    serial_printf("temp = %04u\n", temp);
    calc_csum();
    return temp;
}

void 
print_temp(unsigned int temp)
{
   int i = 0;
   byte val;
   byte led_digits[4] = { 0xc0, 0xc2, 0xc4, 0xc6 };
   LED.sendNum((int)temp, 0);
}

void loop() 
{
    int x;
    unsigned int temp;

    // Write data bytes to serial
    for (x = 0; x <= 6; x++) {
        ecu.write(sbytes[x]);
        delay(10);
    }
    ecu.write(sbytes[7]);

    // Intermessage delay
    delay(30);
    if (digitalRead(UP_BUTTON)) {
        temp = increment_temp();
        print_temp(temp);
        serial_printf("%02x %02x %02x %02x %02x %02x %02x %02x\n", 
                sbytes[0], sbytes[1], sbytes[2], sbytes[3], 
                sbytes[4], sbytes[5], sbytes[6], sbytes[7]);
   
    } else if (digitalRead(DOWN_BUTTON)) {
        temp = decrement_temp();
        print_temp(temp);
        serial_printf("%02x %02x %02x %02x %02x %02x %02x %02x\n", 
                sbytes[0], sbytes[1], sbytes[2], sbytes[3], 
                sbytes[4], sbytes[5], sbytes[6], sbytes[7]);
   
    }
}
