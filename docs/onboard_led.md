# On-board LED

Controls the on-board LED taking in to account the difference with the WiFi enabled Pico.

## led_init

`int led_init(void)`

Initialises the on-board led and returns 0 if no error, or an error code.


## led_set

`void led_set(bool on)`

Lights or unlights the on-board LED.

### Parameters

- on - true to light the LED



