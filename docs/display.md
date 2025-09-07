# Display

The display driver emulates an ANSI terminal.  

This driver uses very little RAM leaving more for your project as a frame buffer in the Pico RAM is not used.

Colours default to plain ASCII (black and phosphor), or you can use ANSI 16-colour, 256-colour palette (216 colors + 16 ANSI + 24 gray) and 24-bit truecolor (approximate to 65K colours). You can chose a between white, green or amber presets as your default phosphor colour, or chose your own.

The UK and [Special Graphics](https://vt100.net/docs/vt100-ug/chapter3.html#T3-9) character sets of the VT100 are supported.

The font (8x10) is easily modifyable in source with out any additional tooling. You draw the glyphs using 1's and 0's:

``` C
    // 0x41
    0b00010000,
    0b00101000,
    0b01000100,
    0b10000010,
    0b11111110,
    0b10000010,
    0b10000010,
    0b00000000,
    0b00000000,
    0b00000000,
```

## display_init

`void display_init(void)`

Initialises the LCD display.


## display_set_led_callback

`void display_set_led_callback(led_callback_t callback)`

Sets a callback function that is called when the state of the LEDs changes.

### Parameters

led_callback – called when the state of the LEDs changes


## display_set_bell_callback

`void display_set_bell_callback(bell_callback_t callback)`

Sets a callback function that is called when the BEL character is received.

### Parameters

bell_callback – called when the BEL character is received


## display_set_report_callback

`void display_set_report_callback(report_callback_t callback)`

Sets a callback function that is called when the REPORT character is received.

### Parameters

report_callback – called when the REPORT character is received


## display_emit_available

`bool display_emit_available(void)`

Returns true if the display can accept characters.


## display_emit

`void display_emit(char c)`

Display a character on the display or processes the ANSI escape sequence.

### Parameters

c – the character to process

