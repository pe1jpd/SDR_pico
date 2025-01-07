
#include "Arduino.h"
#include <stdio.h> // dec2bcd
#include <Wire.h>
#include <SPI.h>
#include "touch.h"
#include "tft.h"
#include "ui.h" // for GP_TOUCH and events
#include "ds3231.h"


#define SPI_TOUCH_FREQUENCY  2500000

extern uint8_t event;
extern bool calibrated;

uint16_t xmin = 220, xmax = 3800;
uint16_t ymin = 320, ymax = 3750;

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


#define I2C_EEPROM   0x50

void i2c_io_write(byte device, byte data)
{
    Wire.beginTransmission(device);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t i2c_io_read(byte device)
{
  uint8_t c = 0xff;
  Wire.beginTransmission(device);
  Wire.requestFrom(device, 1);
    if (Wire.available()) 
      c  = Wire.read();
  return c;
}

void i2c_eeprom_write(int address, byte *data, int len)
{
    Wire.beginTransmission(I2C_EEPROM);
    Wire.write((uint8_t)(address >> 8)); // MSB
    Wire.write((uint8_t)(address & 0xFF)); // LSB
    while (len--)
      Wire.write(*data++);
    Wire.endTransmission();
    delay(100);
}

void i2c_eeprom_read(int address, byte *data, int len)
{
    Wire.beginTransmission(I2C_EEPROM);
    Wire.write((uint8_t)(address >> 8)); // MSB
    Wire.write((uint8_t)(address & 0xFF)); // LSB
    Wire.endTransmission(false);

    Wire.requestFrom(I2C_EEPROM, len);
    while (len--) {
      if (Wire.available()) 
        *data++  = Wire.read();
    }
//    Wire.endTransmission(true);
}

void getTouch(uint16_t *x, uint16_t *y)
{
  char s[20];
  int i;
  uint16_t xp, yp, zp;
  uint32_t xt=0, yt=0, zt=0;

  for (i=0; i<10; i++) {
    if (gpio_get(GP_TOUCH)==1) break;
    getTouchRaw(&xp, &yp, &zp);
    xt += (uint32_t)xp;
    yt += (uint32_t)yp;
    zt += (uint32_t)zp;
  }
  xp = xt/i; yp = yt/i; zp = zt/i;
/*
  setFont(1);
  sprintf(s, "%d,%d,%d   ", xp, yp, zp);
  drawString(s, CWLINEX, CWLINEY, 1, YELLOW, BLACK);
 */
  if (zp>10+xp/400) {                                         // x depends on pressure z.
    *x = map(xp, xmin, xmax, 480, 0);
    *y = map(yp, ymin, ymax, 0, 272);
  }
}

void getTouchRaw(uint16_t *xp, uint16_t *yp, uint16_t *zp)
{
  int tmp;
  int z, z1, z2;

  digitalWrite(T_CS, LOW);
  SPI.beginTransaction(SPISettings(SPI_TOUCH_FREQUENCY, MSBFIRST, SPI_MODE0));

  SPI.transfer(0xb1);                    // Start new Z conversion
  tmp = SPI.transfer(0xc1)>>3;           // Start new Z conversion
  *zp = tmp;

  // Start YP sample request for x position, read 4 times and keep last sample
  SPI.transfer(0xd0);                    // Start new YP conversion
  SPI.transfer(0);                       // Read first 8 bits
  SPI.transfer(0xd0);                    // Read last 8 bits and start new YP conversion
  SPI.transfer(0);                       // Read first 8 bits
  SPI.transfer(0xd0);                    // Read last 8 bits and start new YP conversion
  SPI.transfer(0);                       // Read first 8 bits
  SPI.transfer(0xd0);                    // Read last 8 bits and start new YP conversion

  tmp = SPI.transfer(0);                   // Read first 8 bits
  tmp = tmp <<5;
  tmp |= 0x1f & (SPI.transfer(0x90)>>3);   // Read last 8 bits and start new XP conversion

  *xp = tmp;

  // Start XP sample request for y position, read 4 times and keep last sample
  SPI.transfer(0);                       // Read first 8 bits
  SPI.transfer(0x90);                    // Read last 8 bits and start new XP conversion
  SPI.transfer(0);                       // Read first 8 bits
  SPI.transfer(0x90);                    // Read last 8 bits and start new XP conversion
  SPI.transfer(0);                       // Read first 8 bits
  SPI.transfer(0x90);                    // Read last 8 bits and start new XP conversion

  tmp = SPI.transfer(0);                 // Read first 8 bits
  tmp = tmp <<5;
  tmp |= 0x1f & (SPI.transfer(0)>>3);    // Read last 8 bits

  *yp = tmp;

  SPI.endTransaction();
  digitalWrite(T_CS, HIGH);
}

uint8_t dec2bcd(uint8_t n)
{
  return (n/10)<<4 +  n%10;
}

uint8_t bcd2dec(uint8_t n)
{
  uint8_t p = 10*(n>>4);
  p += n & 0x0f;
  return p;
}

void DS3231_set(struct ts t)
{
    Wire.beginTransmission(I2C_DS3231);
    Wire.write(0x00);
    delay(50);
    Wire.write(t.sec);
    delay(50);
    Wire.write(t.min);
    delay(50);
    Wire.write(t.hour);
    delay(50);
    Wire.write(t.wday);
    delay(50);
    Wire.write(t.mday);
    delay(50);
    Wire.write(t.mon);
    delay(50);
    Wire.write(t.year);
    delay(50);
    Wire.endTransmission();
    delay(50);
}

void rtctest()
{
  struct ts t;

  t.sec  = 0x00;
  t.min  = 0x26;
  t.hour = 0x18;
  t.wday = 0x05;
  t.mday = 0x05;
  t.mon  = 0x10;
  t.year = 0x23;
  DS3231_set(t);
}

void DS3231_get(struct ts *t)
{
  Wire.beginTransmission(I2C_DS3231);
  Wire.write(0x00);
  t->hour = t->min = t->sec = 0;
  if (Wire.endTransmission() == 0) {
    Wire.requestFrom(I2C_DS3231, 7);
    t->sec  = bcd2dec(Wire.read());
    t->min  = bcd2dec(Wire.read());
    t->hour = bcd2dec(Wire.read()) & 0x1f;
    t->wday = bcd2dec(Wire.read());
    t->mday = bcd2dec(Wire.read());
    t->mon  = bcd2dec(Wire.read()) & 0x7f;
    t->year = bcd2dec(Wire.read());
  }
}

uint8_t getByte(char *buf)
{
  uint8_t c, n=0;

  for (int i=0; i<2; i++) {
    c = *buf++ - '0';
    n <<= 4;
    n += c;
  } 
  return n;
}
/*
void setTime()
{
  int i=0;
  struct ts t;
  char c, buf[32];

  Serial.println("Enter time in format year/month/day hour:min:sec");
  while (c != 0x0d) {
    if (Serial.available()) {
      c = Serial.read();
      if (c>='0' && c<='9')
  	      buf[i++] = c;
    }
  }

  bufptr = &buf[0];
  t.year = getByte();
  t.mon  = getByte();
  t.mday = getByte();
  t.hour = getByte();
  t.min  = getByte();
  t.sec  = getByte();

  DS3231_set(t);
}
*/


