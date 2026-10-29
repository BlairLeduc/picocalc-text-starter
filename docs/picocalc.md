# PicoCalc

This pseudo driver configures the southbridge, display, keyboard, audio and file system drivers. The display and keyboard are connected to the C stdio <stdio.h> library (printf, scanf, getchar, putchar, ...).

## picocalc_init

`void picocalc_init(void)`

Initialise the southbridge, display, keyboard and audio, and set up the SD card and [file system](filesystem.md) (`fs_init()`). Connects the C stdio functions to the display and keyboard.


