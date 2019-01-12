/*
 * 
 * from https://github.com/kgoba/ft8_lib
  little changes for Arduino :
  - kGenerator table into progmem to save RAM space
  - RTC
  - ad9833
    73 F4GOH@orange.fr 
	
	
  - T/R sequence length: 15 s
  - Message length: 75 bits + 12-bit CRC
  - FEC code: LDPC(174,87)
  - Modulation: 8-FSK, tone spacing 6.25 Hz   ---->> useful
  - Constant-envelope waveform
  - Occupied bandwidth: 50 Hz
  - Synchronization: 7x7 Costas arrays at start, middle, and end
  - Transmission duration: 79*1920/12000 = 12.64 s
  - interval 1920/12000=0.16s                ---->> useful
  - Decoding threshold: -20 dB; several dB lower with AP decoding
  - Multi-decoder finds and decodes all FT8 signals in passband
  - Optional auto-sequencing and auto-reply to a CQ response
  - Operational behavior similar to JT9, JT65
  
  ad9833
  PGA mcp410101.pdf

*/


#include <SPI.h>
#include <DS3232RTC.h> //http://github.com/JChristensen/DS3232RTC
#include <Wire.h>
#include <Time.h>
#include "constants.h"
#include "pack.h"
#include "encode.h"





// AD9833 Control Register helpers
#define CR_B28_COMBINED      0x2000
#define CR_FSELECT_0         0x0000
#define CR_PSELECT_0         0x0000
#define CR_RESET             0x0000
#define CR_SLEEP1            0x0080
#define CR_SLEEP12           0x0040
#define CR_OPBITEN           0x0020
#define CR_DIV2              0x0008
#define CR_MODE_D1_TRIANGLE  0x0002
#define CR_MODE_D1_SINE      0x0000

// Mnemonics for wave forms
#define SINE                 (CR_B28_COMBINED | CR_MODE_D1_SINE)
#define SQUARE               (CR_B28_COMBINED | CR_OPBITEN)
#define FAST_SQUARE          (SQUARE | CR_DIV2)
#define TRIANGLE             (CR_B28_COMBINED | CR_MODE_D1_TRIANGLE)

#define FREQ0                0x4000
#define PHASE0               0xC000
#define REF_FREQ             25000000.0
#define SPI_CLOCK_SPEED 12000000


#define FNC 10 //FSY
#define CS 9
#define SCK 13
#define MOSI 11




#define PC_BAUD_RATE  9600

double freq = 7075000;

uint8_t tones[ft8::NN];          // FT8_NN = 79, lack of better name at the moment

tmElements_t tm;

// the setup routine runs once when you press reset:
void setup() {

  Serial.begin(PC_BAUD_RATE);
  pinMode(FNC, OUTPUT);
  digitalWrite(FNC, HIGH);
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE2);

  setPGA(100);     //0 to 100 %
  setfreq(0, 0);


  const char *message = "CQ F4GOH JN07";
  //  const char *message = "TEST";

  Serial.println(message);


  // First, pack the text data into binary message
  uint8_t packed[ft8::K_BYTES];
  int rc = ft8::pack77(message, packed);
  if (rc < 0) {
    Serial.println("Cannot parse message!\n");
    Serial.println(rc);
    //return -2;
  }


  Serial.println("Packed data: ");
  for (int j = 0; j < 10; ++j) {
    Serial.print(packed[j], HEX);
    Serial.print(",");

  }
  Serial.println();

  ft8::genft8(packed, tones);

  Serial.println("FSK tones: ");
  for (int j = 0; j < ft8::NN; ++j) {
    Serial.print(tones[j]);
    //Serial.print(",");
  }
  Serial.println();
 // majRtc();  //enable to update RTC

}

// the loop routine runs over and over again forever:
void loop() {

  RTC.read(tm);

  if (tm.Second%15 == 0) {
    sendFt8();   // send every 15 seconds
  }
  Serial.print(tm.Hour);
  Serial.print(":");
  Serial.print(tm.Minute);
  Serial.print(":");
  Serial.println(tm.Second);


  char c;
 
  if (Serial.available() > 0) {   //send ft8 manually
    c = Serial.read();
    if (c == 't') {
      sendFt8();
    }
  }
delay(100);
}

void setfreq(double f, uint16_t p) {      //control dds as9833
  uint32_t deltaphase;

  deltaphase =  (f * 268435456) / 25000000;
  int freq_MSB = (int)(deltaphase >> 14) | FREQ0;
  int freq_LSB = (int)(deltaphase & 0x3FFF) | FREQ0;

  digitalWrite(FNC, LOW);
  SPI.transfer16(CR_B28_COMBINED | CR_FSELECT_0 | CR_PSELECT_0 | CR_RESET);
  SPI.transfer16(freq_LSB);
  SPI.transfer16(freq_MSB);
  SPI.transfer16(PHASE0 | (p << 7));
  SPI.transfer16(SINE);
  digitalWrite(FNC, HIGH);
}

void setPGA(int level)      //ctrl PGA
{
  int lv = (level * 211) / 100;
  digitalWrite(CS, LOW);
  SPI.transfer16(level | 0x1100);
  digitalWrite(CS, HIGH);
}

void sendFt8() {        //send ft8 tones
  int n;
  float s;
  for (n = 0; n < ft8::NN; n++) {
    s = (float) tones[n];
    setfreq(freq + (s * 6.25), 0);
    delay(160);
    Serial.print(s);
    Serial.print(',');
  }
  Serial.println();
  setfreq(0, 0);
}

void majRtc()   //update RTC
{
  time_t t;
  //check for input to set the RTC, minimum length is 12, i.e. yy,m,d,h,m,s
  Serial.println(F("update time : format yy,m,d,h,m,s"));
  Serial.println(F("exemple : 2016,6,18,16,32,30, "));

  while (Serial.available () <= 12) {
  }
  //note that the tmElements_t Year member is an offset from 1970,
  //but the RTC wants the last two digits of the calendar year.
  //use the convenience macros from Time.h to do the conversions.
  int y = Serial.parseInt();
  if (y >= 100 && y < 1000)
    Serial.println(F("Error: Year must be two digits or four digits!"));
  else {
    if (y >= 1000)
      tm.Year = CalendarYrToTm(y);
    else    //(y < 100)
      tm.Year = y2kYearToTm(y);
    tm.Month = Serial.parseInt();
    tm.Day = Serial.parseInt();
    tm.Hour = Serial.parseInt();
    tm.Minute = Serial.parseInt();
    tm.Second = Serial.parseInt();
    t = makeTime(tm);
    RTC.set(t);        //use the time_t value to ensure correct weekday is set
    setTime(t);
    while (Serial.available () > 0) Serial.read();
  }
}





