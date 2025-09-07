# LCD

The driver for the LCD display is optimised for displaying text. For performance, the display is operated in 65K colour depth (RGB565) and any colour from this colour depth can be chosen and used. 

This driver uses very little RAM leaving more for your project as a frame buffer in the Pico RAM is not used.

## lcd_init

`void lcd_init(void)`

Initialise the LCD controller.


## lcd_set_colour

`void lcd_set_foreground(uint16_t colour)`

Set the foreground colour for text.

### Parameters

- colour – the RGB565 colour to use for text


## lcd_set_background

`void lcd_set_background(uint16_t colour)`

Set the background colour for text.

### Parameters

- colour – the RGB565 colour to use for text


## lcd_set_reverse

`void lcd_set_reverse(bool reverse_on)`

Set the reverse video mode for text. In reverse video mode, the foreground colour is used for the background and the background colour is used for the text.

### Parameters

- reverse_on – true to enable reverse video mode, false to disable



## lcd_set_underscore

`void lcd_set_underscore(bool underscore_on)`

Set the underscore mode for text. In underscore mode, an underscore is drawn under the text.

### Parameters

- underscore_on – true to enable underscore mode, false to disable



## lcd_set_bold

`void lcd_set_bold(bool bold_on)`

Set the bold mode for text. In bold mode, the text is drawn with a thicker font.

### Parameters

- bold_on – true to enable bold mode, false to disable


## lcd_set_font

`void lcd_set_font(const font_t *new_font)`

Sets the font to use for text.

### Parameters

- new_font – pointer to the new font to use


## lcd_get_columns

`uint8_t lcd_get_columns(void)`

Returns the number of columns in the display.


## lcd_get_glyph_width

`uint8_t lcd_get_glyph_width(void)`

Returns the width of a glyph in pixels.


## lcd_display_on

`void lcd_display_on(void)`

Turn the display on.


## lcd_display_off

`void lcd_display_off(void)`

Turn the display off.


## lcd_blit

`void lcd_blit(uint16_t *pixels, uint16_t x, uint16_t y, uint16_t width, uint16_t height)`

Writes pixel data to a region of the frame buffer in the display controller and takes into account the scrolled display.

### Parameters

- pixels – array of pixels (RGB565)
- x – left edge corner of the region in pixels
- y – top edge of the region in pixels
- width – width of the region in pixels
- height - height of the region in pixels


## lcd_solid_rectangle

`void lcd_solid_rectangle(uint16_t colour, uint16_t x, uint16_t y, uint16_t width, uint16_t height)`

Draws a solid rectangle using a single colour.

### Parameters

- colour – the RGB565 colour
- x – left edge corner of the rectangle in pixels
- y – top edge of the rectangle in pixels
- width – width of the rectangle in pixels
- height - height of the rectangle in pixels


## lcd_define_scrolling

`void lcd_define_scrolling(uint16_t top_fixed_area, uint16_t bottom_fixed_area)`

Define the area that will be scrolled on the display. The scrollable area is between the top fixed area and the bottom fixed area.

### Parameters

- top_fixed_area – Number of pixel rows fixed at the top of the display
- bottom_fixed_area – Number of pixel rows fixed at the bottom of the display


## lcd_scroll_up

`void lcd_scroll_up(void)`

Scroll the screen up one line adding room for a line of text at the bottom of the scrollable area.


## lcd_scroll_down

`void lcd_scroll_down(void)`

Scroll the screen down one line adding room for a line of text at the top of the scrollable area.


## lcd_clear_screen

`void lcd_clear_screen(void)`

Clear the display.


## lcd_erase_line

`void lcd_erase_line(uint8_t row, uint8_t col_start, uint8_t col_end)`

Erases a line of text on the display.

### Parameters

- row – the row to erase
- col_start – the starting column to erase
- col_end – the ending column to erase


## lcd_putc

`void lcd_putc(uint8_t column, uint8_t row, uint8_t c)`

Draws a glyph at a location on the display.

### Parameters

- column - horizontal location to draw
- row – vertical location to draw
- c – glygh to draw (font offset)


## lcd_putstr

`void lcd_putstr(uint8_t column, uint8_t row, const char *s)`

Draws a string of glyphs at a location on the display.

### Parameters

- column - horizontal location to draw
- row – vertical location to draw
- s – a pointer to the string of glyphs to draw (font offsets)


## lcd_move_cursor

`void lcd_move_cursor(uint8_t column, uint8_t row)`

Move to cursor to a location.

### Parameters

- column - horizontal location to draw
- row – vertical location to draw


## lcd_draw_cursor

`void lcd_draw_cursor(void)`

Draws the cursor, if enabled.


## lcd_erase_cursor

`void lcd_erase_cursor(void)`

Erases the cursor, if enabled.


## lcd_enable_cursor

`void lcd_enable_cursor(bool cursor_on)`

Enable or disable the cursor.


## lcd_cursor_enabled

`bool lcd_cursor_enabled(void)`

Determine if the cursor is enabled.
