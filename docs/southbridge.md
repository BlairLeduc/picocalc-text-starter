# Southbridge

The southbridge is the MPU (STM32F103R8T6) on the mainboard of the PicoCalc. This MPU interfaces the low-speed devices to the Pico.

## sb_read_keyboard

`uint16_t sb_read_keyboard(void)`

Read a key status and code from the keyboard as a 16-bit half-word. The upper byte is the key status, the lower byte is the key code. 

## sb_read_keyboard_state

`uint16_t sb_read_keyboard_state(void)`

Read the current state of the keyboard. The value is the number of characters waiting in the FIFO. If bit 5 is set, it indicates that the CapsLK is set.

## sb_read_battery

`uint8_t sb_read_battery(void)`

Read the battery status. The MSB of the returned valus is set if the battery is charging.

## sb_read_lcd_backlight

`uint8_t sb_read_lcd_backlight(void)`

Read the current LCD Display backlight brightness, 0 (dark) to 255 (bright).


## sb_write_lcd_backlight

`void sb_write_lcd_backlight(uint8_t brightness)`

Sets the LCD Display backlight brightness.

### Parameters

- brightness – a value between 0 (dark) and 255 (bright)


## sb_read_keyboard_backlight

`uint8_t sb_read_keyboard_backlight(void)`

Reads the current keyboard backlight brightness, 0 (dark) to 255 (bright).


## sb_write_keyboard_backlight

`void sb_write_keyboard_backlight(uint8_t brightness)`

Sets the keyboard backlight brightness.

### Parameters

- brightness – a value between 0 (dark) and 255 (bright)


## sb_is_power_off_supported

`bool sb_is_power_off_supported(void)`

Returns true if power off is supported, false otherwise.


## sb_write_power_off_delay

`void sb_write_power_off_delay(uint8_t delay_seconds)`

Sets a power off delay with the power management unit. Not sure what this does.


## sb_reset

`void sb_reset(uint8_t delay_seconds)`

Reset the PicoCalc after a delay.
