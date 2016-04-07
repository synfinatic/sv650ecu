/*
  ST6961.cpp - Library for communicating with ST6961 LED driver.
  Created by Garrett Blanton January, 16, 2013.
  Released into the public domain.
*/

//0 - A
//1 - B
//2 - C
//3 - D
//4 - E
//5 - F
//6 - G
//7 - Colon

#include "Arduino.h"
#include "ST6961.h"

ST6961::ST6961(int DIN, int CLK, int CS)
{
  pinMode(DIN, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(CS, OUTPUT);
  _DIN = DIN;
  _CLK = CLK;
  _CS = CS;
}

void ST6961::initDisplay()
{
  sendCmd(_DISPLAY_6X12);
  sendCmd(_AUTO_INCREMENT);
  initRAM();
  sendCmd(_DISPLAY_14_16);
}

// Initializes RAM to all zeros
void ST6961::initRAM()
{
  //first clear 8 bytes of the display RAM
  digitalWrite(_CS,LOW);
  shiftOut(_DIN, _CLK, LSBFIRST, 0xC0);
  for(int i=0; i<8; i++){
    shiftOut(_DIN, _CLK, LSBFIRST, 0x00);
  }
  digitalWrite(_CS,HIGH); 
}

// Use to send command based on enumeration
void ST6961::sendCmd(char cmd)
{
  digitalWrite(_CS,LOW);
  shiftOut(_DIN, _CLK, LSBFIRST, cmd);
  digitalWrite(_CS,HIGH);  
}

void ST6961::sendDigit(char digit, char val)
{
  digitalWrite(_CS,LOW);
  shiftOut(_DIN, _CLK, LSBFIRST, digit);
  shiftOut(_DIN, _CLK, LSBFIRST, val);
  digitalWrite(_CS,HIGH);    
}

void ST6961::sendNum(int num, char colon)
{
  int digit1 = num / 1000;
  int digit2 = (num % 1000) / 100;
  int digit3 = (num % 100) / 10;
  int digit4 = (num % 10);
  
  sendDigits(digit1,digit2,digit3,digit4,colon);
}

void ST6961::sendDigits(char digit1, char digit2, char digit3, char digit4, char colon)
{

  const char DISP[16] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x58, 0x5e, 0x79, 0x71};
  
  digitalWrite(_CS,LOW);
  shiftOut(_DIN, _CLK, LSBFIRST, 0xC0);
  
  if(colon == 1)
  {
    shiftOut(_DIN, _CLK, LSBFIRST, DISP[digit1] | 0x80);
  }
  else
  {
    shiftOut(_DIN, _CLK, LSBFIRST, DISP[digit1]);
  }

  shiftOut(_DIN, _CLK, LSBFIRST, 0xC2);

  if(colon == 1)  
  {
    shiftOut(_DIN, _CLK, LSBFIRST, DISP[digit2] | 0x80);
  }
  else
  {
    shiftOut(_DIN, _CLK, LSBFIRST, DISP[digit2]);
  }
  
  shiftOut(_DIN, _CLK, LSBFIRST, 0xC4);
  shiftOut(_DIN, _CLK, LSBFIRST, DISP[digit3]);
  shiftOut(_DIN, _CLK, LSBFIRST, 0xC6);
  shiftOut(_DIN, _CLK, LSBFIRST, DISP[digit4]);
  digitalWrite(_CS,HIGH);    
}