/* Stand-in for the Pico SDK header, for sd_busy_probe.c only: the functions are
 * implemented by the fake SD card in that file. */
#ifndef STUB_SD_HARDWARE_SPI_H
#define STUB_SD_HARDWARE_SPI_H

#include <stddef.h>
#include <stdint.h>

#define spi0 0

unsigned spi_init(int spi, unsigned baud);
unsigned spi_set_baudrate(int spi, unsigned baud);
int spi_write_blocking(int spi, const uint8_t *src, size_t len);
int spi_write_read_blocking(int spi, const uint8_t *src, uint8_t *dst, size_t len);

#endif
