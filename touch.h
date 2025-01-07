#ifndef _TOUCH_H
#define _TOUCH_H

#ifdef __cplusplus
 extern "C" {
#endif



// Pico pins used
#define T_DO  16 
#define T_CS  17
#define T_SCK 18
#define T_DI  19



// prototypes
void getTouch(uint16_t *x, uint16_t *y);
void getTouchRaw(uint16_t *x, uint16_t *y, uint16_t *z);
uint8_t getByte(char *buf);
void i2c_io_write(byte device, byte data);
uint8_t i2c_io_read(byte device);
void i2c_eeprom_write(int address, byte *data, int len);
void i2c_eeprom_read(int address, byte *data, int len);

#ifdef __cplusplus
}
#endif
#endif /* _TOUCH_H */