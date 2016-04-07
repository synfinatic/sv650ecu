/*
  ST6961.h - Library for communicating with ST6961 LED driver.
  Created by Garrett Blanton January, 16, 2013.
  Released into the public domain.
*/

#ifndef ST6961_h
#define ST6961_h

#include "Arduino.h"

class ST6961
{
  public:
  
    ST6961(int DIN, int CLK, int CS);
    void initDisplay();
    void initRAM();
    void sendCmd(char cmd);
    void sendDigit(char digit, char val);
    void sendNum(int num, char colon);
    void sendDigits(char digit1, char digit2, char digit3, char digit4, char colon);
	
    const static char _DISPLAY_6X12 = 0x02;
    const static char _DISPLAY_7X11 = 0x03;
    const static char _AUTO_INCREMENT = 0x40;
    const static char _FIXED_ADDRESS = 0x44;
    const static char _DISPLAY_OFF = 0x80;
    const static char _DISPLAY_1_16 = 0x88;
    const static char _DISPLAY_2_16 = 0x89;
    const static char _DISPLAY_4_16 = 0x8A;
    const static char _DISPLAY_10_16 = 0x8B;
    const static char _DISPLAY_11_16 = 0x8C;
    const static char _DISPLAY_12_16 = 0x8D;
    const static char _DISPLAY_13_16 = 0x8E;
    const static char _DISPLAY_14_16 = 0x8F;
	
  private:
  
	int _DIN;
    int _CLK;
    int _CS;
	
};

#endif