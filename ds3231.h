#ifndef __ds3231_h_
#define __ds3231_h_

#ifdef __cplusplus
 extern "C" {
#endif

#include <Arduino.h>
#include "config.h"

#define	DS3231_TRANSACTION_TIMEOUT	100 // Wire NAK/Busy timeout in ms

#define SECONDS_FROM_1970_TO_2000   946684800

// i2c slave address of the DS3231 chip
#define I2C_DS3231                  0x68

struct ts {
    uint8_t sec;         /* seconds */
    uint8_t min;         /* minutes */
    uint8_t hour;        /* hours */
    uint8_t mday;        /* day of the month */
    uint8_t mon;         /* month */
    int16_t year;        /* year */
    uint8_t wday;        /* day of the week */
    uint8_t yday;        /* day in the year */
    uint8_t isdst;       /* daylight saving time */
    uint8_t year_s;      /* year in short notation*/
#ifdef CONFIG_UNIXTIME
    uint32_t unixtime;   /* seconds since 01.01.1970 00:00:00 UTC*/
#endif
};

void DS3231_init(const uint8_t creg);
void DS3231_set(struct ts t);
void DS3231_get(struct ts *t);
void setTime();
void rtctest();

/*
void DS3231_set_addr(const uint8_t addr, const uint8_t val);
uint8_t DS3231_get_addr(const uint8_t addr);

// control/status register
void DS3231_set_creg(const uint8_t val);
uint8_t DS3231_get_creg(void);

void DS3231_set_sreg(const uint8_t val);
uint8_t DS3231_get_sreg(void);

// aging offset register
void DS3231_set_aging(const int8_t val);
int8_t DS3231_get_aging(void);

// temperature register
float DS3231_get_treg(void);

void DS3231_set_32kHz_output(const uint8_t on);

// alarms
void DS3231_set_a1(const uint8_t s, const uint8_t mi, const uint8_t h, const uint8_t d,
                   const uint8_t * flags);
void DS3231_get_a1(char *buf, const uint8_t len);
void DS3231_clear_a1f(void);
uint8_t DS3231_triggered_a1(void);

void DS3231_set_a2(const uint8_t mi, const uint8_t h, const uint8_t d, const uint8_t * flags);
void DS3231_get_a2(char *buf, const uint8_t len);
void DS3231_clear_a2f(void);
uint8_t DS3231_triggered_a2(void);

*/
// helpers
uint32_t get_unixtime(struct ts t);
uint8_t dectobcd(const uint8_t val);
uint8_t bcdtodec(const uint8_t val);
uint8_t inp2toi(char *cmd, const uint16_t seek);

#ifdef __cplusplus
}
#endif
#endif