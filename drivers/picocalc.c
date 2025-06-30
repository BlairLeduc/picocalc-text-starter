#include "stdio.h"
#include "pico/stdlib.h"
#include "pico/stdio/driver.h"

#include "display.h"
#include "keyboard.h"
#include "southbridge.h"

// Callback for when characters become available
static void (*chars_available_callback)(void *) = NULL;
static void *chars_available_param = NULL;

static void picocalc_out_chars(const char *buf, int length)
{
    for (int i = 0; i < length; ++i)
    {
        display_emit(buf[i]);
    }
}

static void picocalc_out_flush(void)
{
    // No flush needed for this driver
}

static int picocalc_in_chars(char *buf, int length)
{
    int n = 0;
    while (n < length)
    {
        int c = keyboard_get_key();
        if (c == -1)
            break; // No key pressed
        buf[n++] = (char)c;
    }
    return n;
}

static void picocalc_set_chars_available_callback(void (*fn)(void *), void *param)
{
    chars_available_callback = fn;
    chars_available_param = param;
}

// Function to be called when characters become available
void picocalc_chars_available_notify(void)
{
    if (chars_available_callback)
    {
        chars_available_callback(chars_available_param);
    }
}

stdio_driver_t my_stdio_driver = {
    .out_chars = picocalc_out_chars,
    .out_flush = picocalc_out_flush,
    .in_chars = picocalc_in_chars,
    .set_chars_available_callback = picocalc_set_chars_available_callback,
    .next = NULL,
};

void picocalc_init(void (*led_set_callback)(uint8_t))
{
    sb_init();
    display_init(led_set_callback);
    keyboard_init(picocalc_chars_available_notify);

    stdio_set_driver_enabled(&my_stdio_driver, true);
    stdio_set_translate_crlf(&my_stdio_driver, true);
}