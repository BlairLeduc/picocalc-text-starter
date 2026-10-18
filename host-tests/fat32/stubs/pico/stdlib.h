/* Minimal stand-in for the Pico SDK header, only so that fat32.c compiles on a PC. */
#ifndef STUB_PICO_STDLIB_H
#define STUB_PICO_STDLIB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct { int unused; } repeating_timer_t;
typedef bool (*repeating_timer_callback_t)(repeating_timer_t *);

static inline bool add_repeating_timer_ms(int ms, repeating_timer_callback_t cb, void *user,
                                          repeating_timer_t *timer)
{
    (void)ms; (void)cb; (void)user; (void)timer;
    return true;
}

static inline void sleep_ms(uint32_t ms) { (void)ms; }
static inline uint64_t time_us_64(void) { return 0; }

#endif
