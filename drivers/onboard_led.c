//
//  LED Driver for the Raspberry Pi Pico
//
//  This driver controls the onboard LED on the Raspberry Pi Pico and
//  can also control the LED on the CYW43 Wi-Fi chip if needed.
//

#include "pico/stdlib.h"
#include "pico/status_led.h"

#include "onboard_led.h"

static bool led_initialised = false; // Flag to indicate if the LED is initialized

// Set the state of the on-board LED
void led_set(bool led)
{
    status_led_set_state(led);
}

// Initialize the LED driver
int led_init()
{
    if (led_initialised) {
        return 0; // Already initialized, return success
    }

    status_led_init();

    led_initialised = true; // Set the initialized flag

    return 0; // Success
}
