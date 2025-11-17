#include "ds3231.h"
#include "southbridge.h"
#include <stdio.h>
#include <stdatomic.h>

// Converte un valore BCD (Binary Coded Decimal) in decimale
uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd / 16) * 10) + (bcd % 16);
}

// Converte un valore decimale in BCD (Binary Coded Decimal)
uint8_t dec_to_bcd(uint8_t dec) {
    return ((dec / 10) * 16) + (dec % 10);
}

bool ds3231_init(void) {
    // Inizializza I2C1 a 100 kHz
    i2c_init(DS3231_I2C_PORT, DS3231_I2C_BAUDRATE);

    // Configura i pin GPIO per I2C
    gpio_set_function(DS3231_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(DS3231_SCL_PIN, GPIO_FUNC_I2C);

    // Abilita i pull-up interni sui pin I2C
    gpio_pull_up(DS3231_SDA_PIN);
    gpio_pull_up(DS3231_SCL_PIN);

    // Verifica che il dispositivo DS3231 sia presente sulla bus I2C
    uint8_t test_data;
    int result = i2c_read_blocking(DS3231_I2C_PORT, DS3231_I2C_ADDR, &test_data, 1, false);

    if (result < 0) {
        printf("Error: DS3231 not found on I2C1 port\n");
        return false;
    }

    printf("DS3231 initialized successfully\n");
    return true;
}

bool ds3231_read_time(ds3231_datetime_t *datetime) {
    if (datetime == NULL) {
        return false;
    }

    // Wait for southbridge I2C to be available
    while (!sb_available()) {
        tight_loop_contents();
    }

    // Acquire I2C lock
    atomic_store(&sb_i2c_in_use, true);

    // Set I2C to DS3231 speed (100kHz)
    i2c_set_baudrate(DS3231_I2C_PORT, DS3231_I2C_BAUDRATE);

    uint8_t buffer[7];
    uint8_t reg = DS3231_REG_SECONDS;

    // Write register address to start reading from
    int result = i2c_write_blocking(DS3231_I2C_PORT, DS3231_I2C_ADDR, &reg, 1, true);
    if (result < 0) {
        printf("Error writing DS3231 register address\n");
        // Restore southbridge I2C speed and release lock
        i2c_set_baudrate(DS3231_I2C_PORT, SB_BAUDRATE);
        atomic_store(&sb_i2c_in_use, false);
        return false;
    }

    // Read 7 bytes (seconds, minutes, hours, day, date, month, year)
    result = i2c_read_blocking(DS3231_I2C_PORT, DS3231_I2C_ADDR, buffer, 7, false);
    if (result < 0) {
        printf("Error reading data from DS3231\n");
        // Restore southbridge I2C speed and release lock
        i2c_set_baudrate(DS3231_I2C_PORT, SB_BAUDRATE);
        atomic_store(&sb_i2c_in_use, false);
        return false;
    }

    // Restore southbridge I2C speed before releasing lock
    i2c_set_baudrate(DS3231_I2C_PORT, SB_BAUDRATE);

    // Release I2C lock
    atomic_store(&sb_i2c_in_use, false);

    // Convert BCD values to decimal and store in structure
    datetime->seconds = bcd_to_dec(buffer[0] & 0x7F);  // Mask bit 7
    datetime->minutes = bcd_to_dec(buffer[1] & 0x7F);  // Mask bit 7
    datetime->hours   = bcd_to_dec(buffer[2] & 0x3F);  // 24-hour format, mask bits 6-7
    datetime->day     = bcd_to_dec(buffer[3] & 0x07);  // Mask bits 3-7
    datetime->date    = bcd_to_dec(buffer[4] & 0x3F);  // Mask bits 6-7
    datetime->month   = bcd_to_dec(buffer[5] & 0x1F);  // Mask bits 5-7
    datetime->year    = bcd_to_dec(buffer[6]);         // Year (0-99)

    return true;
}

bool ds3231_write_time(const ds3231_datetime_t *datetime) {
    if (datetime == NULL) {
        return false;
    }

    // Validate values
    if (datetime->seconds > 59 || datetime->minutes > 59 || datetime->hours > 23 ||
        datetime->day < 1 || datetime->day > 7 ||
        datetime->date < 1 || datetime->date > 31 ||
        datetime->month < 1 || datetime->month > 12 ||
        datetime->year > 99) {
        printf("Error: Invalid date/time values\n");
        return false;
    }

    // Wait for southbridge I2C to be available
    while (!sb_available()) {
        tight_loop_contents();
    }

    // Acquire I2C lock
    atomic_store(&sb_i2c_in_use, true);

    // Set I2C to DS3231 speed (100kHz)
    i2c_set_baudrate(DS3231_I2C_PORT, DS3231_I2C_BAUDRATE);

    // Buffer: [start register, seconds, minutes, hours, day, date, month, year]
    uint8_t buffer[8];

    buffer[0] = DS3231_REG_SECONDS;                // Start register
    buffer[1] = dec_to_bcd(datetime->seconds);     // Seconds in BCD
    buffer[2] = dec_to_bcd(datetime->minutes);     // Minutes in BCD
    buffer[3] = dec_to_bcd(datetime->hours);       // Hours in BCD (24-hour format)
    buffer[4] = dec_to_bcd(datetime->day);         // Day of week in BCD
    buffer[5] = dec_to_bcd(datetime->date);        // Date in BCD
    buffer[6] = dec_to_bcd(datetime->month);       // Month in BCD
    buffer[7] = dec_to_bcd(datetime->year);        // Year in BCD

    // Write all registers in a single transaction
    int result = i2c_write_blocking(DS3231_I2C_PORT, DS3231_I2C_ADDR, buffer, 8, false);

    // Restore southbridge I2C speed
    i2c_set_baudrate(DS3231_I2C_PORT, SB_BAUDRATE);

    // Release I2C lock
    atomic_store(&sb_i2c_in_use, false);

    if (result < 0) {
        printf("Error writing data to DS3231\n");
        return false;
    }

    return true;
}
