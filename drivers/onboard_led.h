//
//  LED Driver for the Raspberry Pi Pico
//  Copyright 2025-2026 Blair Leduc. See LICENSE for details.
//

#pragma once

#include "pico/stdlib.h"

int led_init(void);
void led_set(bool on);