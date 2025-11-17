#ifndef DS3231_H
#define DS3231_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// DS3231 I2C Address
#define DS3231_I2C_ADDR 0x68

// DS3231 Register Addresses
#define DS3231_REG_SECONDS      0x00
#define DS3231_REG_MINUTES      0x01
#define DS3231_REG_HOURS        0x02
#define DS3231_REG_DAY          0x03
#define DS3231_REG_DATE         0x04
#define DS3231_REG_MONTH        0x05
#define DS3231_REG_YEAR         0x06
#define DS3231_REG_CONTROL      0x0E
#define DS3231_REG_STATUS       0x0F
#define DS3231_REG_TEMP_MSB     0x11
#define DS3231_REG_TEMP_LSB     0x12

// I2C Configuration
#define DS3231_I2C_PORT         i2c1
#define DS3231_SDA_PIN          6
#define DS3231_SCL_PIN          7
#define DS3231_I2C_BAUDRATE     100000  // 100 kHz

// Structure for date and time
typedef struct {
    uint8_t seconds;    // 0-59
    uint8_t minutes;    // 0-59
    uint8_t hours;      // 0-23 (24-hour format)
    uint8_t day;        // 1-7 (day of week)
    uint8_t date;       // 1-31 (day of month)
    uint8_t month;      // 1-12
    uint8_t year;       // 0-99 (year from 2000)
} ds3231_datetime_t;

/**
 * @brief Initialize DS3231 device on I2C1 port
 *
 * Configure pins GP6 (SDA) and GP7 (SCL) for I2C1 and initialize
 * communication with DS3231 at 100 kHz.
 *
 * @return true if initialization succeeded, false otherwise
 */
bool ds3231_init(void);

/**
 * @brief Read date and time from DS3231
 *
 * Read all time registers from DS3231 and convert them
 * from BCD format to decimal format.
 *
 * @param datetime Pointer to structure where date and time will be saved
 * @return true if read succeeded, false otherwise
 */
bool ds3231_read_time(ds3231_datetime_t *datetime);

/**
 * @brief Write date and time to DS3231
 *
 * Convert values from structure from decimal format to BCD format
 * and write them to DS3231 registers.
 *
 * @param datetime Pointer to structure containing date and time to set
 * @return true if write succeeded, false otherwise
 */
bool ds3231_write_time(const ds3231_datetime_t *datetime);

/**
 * @brief Convert BCD value to decimal
 *
 * @param bcd Value in BCD format
 * @return Value in decimal format
 */
uint8_t bcd_to_dec(uint8_t bcd);

/**
 * @brief Convert decimal value to BCD
 *
 * @param dec Value in decimal format
 * @return Value in BCD format
 */
uint8_t dec_to_bcd(uint8_t dec);

#endif // DS3231_H
