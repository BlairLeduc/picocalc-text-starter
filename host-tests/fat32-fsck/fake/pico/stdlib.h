#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
typedef struct { int unused; } repeating_timer_t;
typedef bool (*repeating_timer_callback_t)(repeating_timer_t *);
static inline bool add_repeating_timer_ms(int32_t ms, repeating_timer_callback_t cb, void *ud, repeating_timer_t *t) { (void)ms; (void)cb; (void)ud; (void)t; return true; }
