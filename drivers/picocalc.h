#pragma once  

#include "pico/stdlib.h"

#include "pico/stdio/driver.h"

typedef void (*led_callback_t)(uint8_t);

extern stdio_driver_t picocalc_stdio_driver;

// Function prototypes
void picocalc_chars_available_notify(void);
void picocalc_init(void);
