/* stand-ins for the Pico SDK, enough for diskio.c, filesystem.c and clib.c */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
typedef uint64_t absolute_time_t;
#define at_the_end_of_time ((absolute_time_t)UINT64_MAX)
typedef struct repeating_timer repeating_timer_t;
typedef bool (*repeating_timer_callback_t)(repeating_timer_t *rt);
struct repeating_timer { repeating_timer_callback_t callback; };
bool add_repeating_timer_ms(int32_t ms, repeating_timer_callback_t cb, void *user, repeating_timer_t *out);
int stdio_get_until(char *buf, int len, absolute_time_t until);
void stdio_put_string(const char *s, int len, bool newline, bool cr_translation);
