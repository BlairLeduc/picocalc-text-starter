/* Stand-in for the Pico SDK header, for sd_busy_probe.c only: the functions are
 * implemented by the fake SD card in that file. */
#ifndef STUB_SD_PICO_STDLIB_H
#define STUB_SD_PICO_STDLIB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GPIO_OUT 1
#define GPIO_IN 0
#define GPIO_FUNC_SPI 1

void gpio_init(unsigned pin);
void gpio_set_dir(unsigned pin, bool out);
void gpio_pull_up(unsigned pin);
void gpio_set_function(unsigned pin, unsigned function);
void gpio_put(unsigned pin, bool value);
bool gpio_get(unsigned pin);
void busy_wait_us(uint32_t us);
void sleep_ms(uint32_t ms);
uint64_t time_us_64(void);

#endif
