# Keyboard

The keyboard driver operates with a timer loop that polls the PicoCalc's southbridge for key presses. Unfortunately, the southbridge cannot notify the Pico when a key is pressed.

The purpose of this implementation was support:

- type ahead
- keyboard user interrupts

The type ahead buffer allows users to type even while your project is processing. When Brk (Shift-Esc) is pressed, a flag is set allowing your project to monitor and stop processing, if desired. 


## keyboard_init

`void keyboard_init(void)`

Initialises the keyboard.


## keyboard_set_key_available_callback

`void keyboard_set_key_available_callback(keyboard_key_available_callback_t callback)`

Sets a callback function that is called when keys are available.

### Parameters

callback - called when keys are available


## keyboard_set_background_poll

`void keyboard_set_background_poll(bool enable)`

Enables or disables background polling of the keyboard.

### Parameters

enable - true to enable background polling, false to disable


## keyboard_poll

`void keyboard_poll(void)`

Polls the keyboard for key presses.


## keyboard_key_available

`bool keyboard_key_available(void)`

Returns true is a key is available.


## keyboard_get_key

`char keyboard_get_key(void)`

Returns a key; blocks if no key is available.


