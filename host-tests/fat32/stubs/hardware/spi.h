/* Minimal stand-in for the Pico SDK header, only so that fat32.c compiles on a PC. */
#ifndef STUB_HARDWARE_SPI_H
#define STUB_HARDWARE_SPI_H

#define spi0 0

static inline unsigned spi_set_baudrate(int spi, unsigned baud)
{
    (void)spi;
    return baud;
}

#endif
