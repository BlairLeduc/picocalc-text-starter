# picocalc-text-starter

With this starter project, you will be able to get started on the [PicoCalc](https://www.clockworkpi.com/picocalc) using the [Pico-Series C/C++ SDK](https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html). You can create the [text-based user interface](https://en.wikipedia.org/wiki/Text-based_user_interface) experience of the 1980's that are well suited for a mouseless system.

This starter contains code to write **text** to the LCD display and read input from the keyboard using the C stdio <stdio.h> library (printf, scanf, getchar, putchar, ...). 

> [!CAUTION]
> This starter is not intended for getting started with only a Raspberry Pi Pico as you should already have, or will need to gain, knowledge of the Pico-series C/C++ SDK and writing embedded code for the Pico-series devices. You will require a [PicoCalc](https://www.clockworkpi.com/product-page/picocalc) to use these drivers.

This starter includes drivers for:

- Audio (stereo)
- Display (multicolour text with ANSI escape code emulation)
- Keyboard
- Pico onboard LED (WiFi-option aware)
- Serial port
- SD Card (read only)
- Southbridge functions (keyboard, battery, backlights)

This starter does not contain the best-of-bread drivers for each component, but rather enough capability to get **your project "started" fast**.

> [!WARNING]
> This starter is not designed, nor intended, to create graphical or sprite-based games. Hopefully, other starters are available that can help you, though you could easily create text-based games.

> [!TIP]
> This is a starter project. Feel free to take bits and pieces and modify what is here to suit ***your*** project. The drivers are independent of each other; cherry-pick what you need for your project. 

# Getting Started

Hello, world!

``` C
#include <stdio.h>

// Include driver headers here
#include "drivers/picocalc.h"

int main()
{
    picocalc_init(NULL);

    // Your project starts here
    printf("Hello, world!\n");
}
```

## Configuration

If you are using [Visual Studio Code](https://code.visualstudio.com) and the [Raspberry Pi Pico](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico) extension, remember to "Switch Board" to the Pico you are using.

If you are using a third-party board with WiFi you will also need to add that board to list of boards that support WiFi near the end of `CMakeList.txt`.

> [!TIP]
> You can manually update `CMakeLists.txt` to set the board to the Pico you are using (PICO_BOARD) and update the WiFi board list at the end of the file. 



# Demo REPL

The main entry point for this starter is a simple REPL to run demos and tests of the drivers and the functioning of your PicoCalc. 

> [!NOTE]
> The REPL is provided only for the demonstration of this library. You will discard and replace the REPL with your project.

## Commands

These commands provide examples of how to use the drivers:

- **backlight** - Displays the backlight values for the display and keyboard
- **battery** – Displays the battery level and status (graphically)
- **beep** – Play a simple beep sound
- **box** – Draws a yellow box using special graphics characters
- **bye** – Reboots the device into BOOTSEL mode
- **cls** – Clears the display
- **cd** – Change the current directory
- **dir** – Display the contents of the current directory
- **free** – Shows the free space remaining on the SD card
- **more** – Display the contents of a file
- **play** – Play a named song (use 'songs' for a list of available songs)
- **pwd** – Displays the current directory
- **sdcard** – Provides information about the inserted SD card
- **songs** – List all available songs
- **test** – Run a named test (use 'tests' for a list of available tests)
- **tests** – List all available tests
- **help** – Lists the available commands

## Songs

A fun song library is provided to give additional testing the audio driver and hardware.

- **baa** – Baa Baa Black Sheep
- **birthday** – Happy Birthday
- **canon** – Canon in D
- **elise** – Fur Elise
- **macdonald** – Old MacDonald Had a Farm
- **mary** – Mary Had a Little Lamb
- **moonlight** – Moonlight Sonata
- **ode** – Ode to Joy (Beethoven)
- **spider** – Itsy Bitsy Spider
- **twinkle** – Twinkle Twinkle Little Star

## Tests

Tests to make sure the hardware and drivers are working correctly.

- **audio** – Test the audio driver with different notes, distinct left/right separation, melodies bouncing between channels, and harmonious intervals. 
- **display** – Display driver stress test with scrolling lines of different colours, writing ANSI escape codes and characters as quickly as possible. Note: characters processed includes the processing of escape squences where characters displayed are the number of characters drawn on the display.


# High-Level Drivers

Documentation for the high-level drivers. These drivers use low-level drivers to function.

- [PicoCalc](docs/picocalc.md) – pseudo driver configures the southbridge, display and keyboard drivers
- [Display](docs/display.md) – emulates an ANSI terminal
- [Keyboard](docs/keyboard.md) – uses a timer loop that polls the PicoCalc's southbridge for key presses
- [FAT32](docs/fat32.md) – read from an SD card formatted with FAT32


# Low-Level Drivers

Documentation for the low-level drivers. These drivers talk directly to the hardware.

- [Audio](docs/audio.md) – simple audio driver can play stereo notes
- [LCD](docs/lcd.md) – driver for the LCD display that is optimised for displaying text
- [On-board LED](docs/onboard_led.md) – controls the on-board LED
- [SD Card](docs/sdcard.md) – driver that allows file systems to talk to the SD card
- [Serial](docs/serial.md) – driver for the USB C serial port
- [Southbridge](docs/southbridge.md) – interfaces to the low-speed devices (keyboard, backlight, battery)
