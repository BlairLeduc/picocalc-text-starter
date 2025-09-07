//
//  PicoCalc LCD display driver
//
//  This driver implements a simple VT100 terminal interface for the PicoCalc LCD display
//  using the ST7789P LCD controller.
//
//  It is optimised for a character-based display with a fixed-width, 8-pixel wide font
//  and 65K colours in the RGB565 format. This driver requires little memory as it
//  uses the frame memory on the controller directly.
//
//  NOTE: Some code below is written to respect timing constraints of the ST7789P controller.
//        For instance, you can usually get away with a short chip select high pulse widths, but
//        writing to the display RAM requires the minimum chip select high pulse width of 40ns.
//

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"

#include "lcd.h"
#include "display.h"

// Colour Palette definitions
//
// The RGB() macro can be used to create colors from this colour depth using 8-bits per
// channel values.

// 3-bit terminal colours (based on VS Code)
const uint16_t palette[8] = {
    RGB(0, 0, 0),      // Black
    RGB(205, 0, 0),    // Red
    RGB(0, 205, 0),    // Green
    RGB(205, 205, 0),  // Yellow
    RGB(0, 0, 238),    // Blue
    RGB(205, 0, 205),  // Magenta
    RGB(0, 205, 205),  // Cyan
    RGB(229, 229, 229) // White
};

// Bright colours for the 3-bit terminal colours (based on VS Code)
const uint16_t bright_palette[8] = {
    RGB(127, 127, 127), // Bright Black (Gray)
    RGB(255, 0, 0),     // Bright Red
    RGB(0, 255, 0),     // Bright Green
    RGB(255, 255, 0),   // Bright Yellow
    RGB(92, 92, 255),   // Bright Blue
    RGB(255, 0, 255),   // Bright Magenta
    RGB(0, 255, 255),   // Bright Cyan
    RGB(255, 255, 255)  // Bright White
};

// Xterm 256-colour palette (RGB565 format)
//
// That is 5-bits for red and blue, and 6-bits for green. The display is configured to map to
// its native 18-bit RGB using the LSB of green for the LSB of red and blue, and the green
// component is unmodified.

const uint16_t xterm_palette[256] = {
    // Standard 16 colors (0-15)
    0x0000, 0x8000, 0x0400, 0x8400, 0x0010, 0x8010, 0x0410, 0xC618,
    0x8410, 0xF800, 0x07E0, 0xFFE0, 0x001F, 0xF81F, 0x07FF, 0xFFFF,

    // 216 colors in 6×6×6 RGB cube (16-231)
    0x0000, 0x0010, 0x0015, 0x001F, 0x0014, 0x001F, 0x0400, 0x0410, 0x0415, 0x041F, 0x0414, 0x041F,
    0x0500, 0x0510, 0x0515, 0x051F, 0x0514, 0x051F, 0x07E0, 0x07F0, 0x07F5, 0x07FF, 0x07F4, 0x07FF,
    0x0600, 0x0610, 0x0615, 0x061F, 0x0614, 0x061F, 0x07E0, 0x07F0, 0x07F5, 0x07FF, 0x07F4, 0x07FF,
    0x8000, 0x8010, 0x8015, 0x801F, 0x8014, 0x801F, 0x8400, 0x8410, 0x8415, 0x841F, 0x8414, 0x841F,
    0x8500, 0x8510, 0x8515, 0x851F, 0x8514, 0x851F, 0x87E0, 0x87F0, 0x87F5, 0x87FF, 0x87F4, 0x87FF,
    0x8600, 0x8610, 0x8615, 0x861F, 0x8614, 0x861F, 0x87E0, 0x87F0, 0x87F5, 0x87FF, 0x87F4, 0x87FF,
    0xA000, 0xA010, 0xA015, 0xA01F, 0xA014, 0xA01F, 0xA400, 0xA410, 0xA415, 0xA41F, 0xA414, 0xA41F,
    0xA500, 0xA510, 0xA515, 0xA51F, 0xA514, 0xA51F, 0xA7E0, 0xA7F0, 0xA7F5, 0xA7FF, 0xA7F4, 0xA7FF,
    0xA600, 0xA610, 0xA615, 0xA61F, 0xA614, 0xA61F, 0xA7E0, 0xA7F0, 0xA7F5, 0xA7FF, 0xA7F4, 0xA7FF,
    0xF800, 0xF810, 0xF815, 0xF81F, 0xF814, 0xF81F, 0xFC00, 0xFC10, 0xFC15, 0xFC1F, 0xFC14, 0xFC1F,
    0xFD00, 0xFD10, 0xFD15, 0xFD1F, 0xFD14, 0xFD1F, 0xFFE0, 0xFFF0, 0xFFF5, 0xFFFF, 0xFFF4, 0xFFFF,
    0xFE00, 0xFE10, 0xFE15, 0xFE1F, 0xFE14, 0xFE1F, 0xFFE0, 0xFFF0, 0xFFF5, 0xFFFF, 0xFFF4, 0xFFFF,
    0xC000, 0xC010, 0xC015, 0xC01F, 0xC014, 0xC01F, 0xC400, 0xC410, 0xC415, 0xC41F, 0xC414, 0xC41F,
    0xC500, 0xC510, 0xC515, 0xC51F, 0xC514, 0xC51F, 0xC7E0, 0xC7F0, 0xC7F5, 0xC7FF, 0xC7F4, 0xC7FF,
    0xC600, 0xC610, 0xC615, 0xC61F, 0xC614, 0xC61F, 0xC7E0, 0xC7F0, 0xC7F5, 0xC7FF, 0xC7F4, 0xC7FF,
    0xE000, 0xE010, 0xE015, 0xE01F, 0xE014, 0xE01F, 0xE400, 0xE410, 0xE415, 0xE41F, 0xE414, 0xE41F,
    0xE500, 0xE510, 0xE515, 0xE51F, 0xE514, 0xE51F, 0xE7E0, 0xE7F0, 0xE7F5, 0xE7FF, 0xE7F4, 0xE7FF,
    0xE600, 0xE610, 0xE615, 0xE61F, 0xE614, 0xE61F, 0xE7E0, 0xE7F0, 0xE7F5, 0xE7FF, 0xE7F4, 0xE7FF,

    // 24 grayscale colors (232-255)
    0x0000, 0x1082, 0x2104, 0x3186, 0x4208, 0x528A, 0x630C, 0x738E,
    0x8410, 0x9492, 0xA514, 0xB596, 0xC618, 0xD69A, 0xE71C, 0xF79E,
    0x0841, 0x18C3, 0x2945, 0x39C7, 0x4A49, 0x5ACB, 0x6B4D, 0x7BCF};

//
//  VT100 Terminal Emulation
//
//  This section contains the definitions and variables used for handling
//  ANSI escape sequences, cursor positioning, and text attributes.
//
//  This implementation is lacking full support for the VT100 terminal.
//
//  Reference: https://vt100.net/docs/vt100-ug/chapter3.html
//

bool tab_stops[64] = {0};
uint8_t debug[64] = {0};
int debug_index = 0;

uint8_t state = STATE_NORMAL; // initial state of escape sequence processing
uint8_t column = 0;           // cursor x position
uint8_t row = 0;              // cursor y position

uint16_t parameters[16]; // buffer for selective parameters
uint8_t p_index = 0;    // index into the buffer

uint8_t save_column = 0; // saved cursor x position for DECSC/DECRC
uint8_t save_row = 0;    // saved cursor y position for DECSC/DECRC
uint8_t leds = 0;        // current LED state

uint8_t g0_charset = CHARSET_ASCII; // G0 character set (default ASCII)
uint8_t g1_charset = CHARSET_ASCII; // G1 character set (default ASCII)
uint8_t active_charset = 0;         // currently active character set (0=G0, 1=G1)

void (*display_led_callback)(uint8_t) = NULL;
void (*display_bell_callback)(void) = NULL;
void (*display_report_callback)(const char *) = NULL;

static void set_g0_charset(uint8_t charset)
{
    g0_charset = charset;
}

static void set_g1_charset(uint8_t charset)
{
    g1_charset = charset;
}

static void set_charset(uint8_t charset)
{
    active_charset = charset; // Set the active character set
}

static uint8_t get_charset()
{
    // Return the currently active character set
    return (active_charset == G0_CHARSET) ? g0_charset : g1_charset;
}

static void update_leds(uint8_t update)
{
    leds = update;

    if (display_led_callback)
    {
        display_led_callback(leds); // Call the user-defined LED callback
    }
}

static void ring_bell()
{
    if (display_bell_callback)
    {
        display_bell_callback(); // Call the user-defined bell callback
    }
}

static void report(const char *msg)
{
    if (display_report_callback)
    {
        display_report_callback(msg);
    }
}

static void reset_terminal()
{
    // Reset terminal state
    lcd_set_reverse(false);
    lcd_set_foreground(FOREGROUND);
    lcd_set_background(BACKGROUND);
    lcd_set_underscore(false);
    lcd_enable_cursor(true);
    set_g0_charset(CHARSET_ASCII); // reset character set to ASCII
    set_g1_charset(CHARSET_ASCII);
    lcd_define_scrolling(0, 0); // no scrolling area defined
    lcd_clear_screen();
    leds = 0;          // reset LED state
    update_leds(leds); // reset LEDs
}

//
// Display API
//

bool display_emit_available()
{
    return true; // always available for output in this implementation
}

void display_emit(char ch)
{
    int max_row = MAX_ROW;
    int max_col = lcd_get_columns() - 1;

    lcd_erase_cursor(); // erase the cursor before processing the character

    // State machine for processing incoming characters
    switch (state)
    {
    case STATE_ESCAPE:        // ESC character received, process the next character
        state = STATE_NORMAL; // reset state by default
        switch (ch)
        {
        case CHR_CAN:                      // cancel the current escape sequence
        case CHR_SUB:                      // same as CAN
            lcd_putc(column++, row, 0x02); // print a error character
            break;
        case CHR_ESC:
            state = STATE_ESCAPE; // stay in escape state
            break;
        case '7': // DECSC – Save Cursor
            save_column = column;
            save_row = row;
            break;
        case '8': // DECRC – Restore Cursor
            column = save_column;
            row = save_row;
            break;
        case 'D': // IND – Index
            row++;
            break;
        case 'E': // NEL – Next Line
            column = 0;
            row++;
            break;
        case 'H': // HTS – Horizontal Tabulation Set
            if (column < sizeof(tab_stops))
            {
                tab_stops[column] = true; // Set a tab stop at the current column
            }
            break;
        case 'M':         // RI – Reverse Index
            if (row == 0) // scroll at top of the screen
            {
                lcd_scroll_down();
            }
            else
            {
                row--;
            }
            break;
        case 'c': // RIS – Reset To Initial State
            column = row = 0;
            reset_terminal();
            break;
        case '[': // CSI - Control Sequence Introducer
            p_index = 0;
            memset(parameters, 0, sizeof(parameters));
            memset(debug, 0, sizeof(debug));
            debug[0] = '\033';
            debug[1] = ch;
            debug_index = 2;
            state = STATE_CS;
            break;
        case ']': // OSC - Operating System Command
        case 'X': // SOS - Start of String (treat as OSC)
        case '^': // PM - Privacy Message (treat as OSC)
        case '_': // APC - Application Program Command (treat as OSC)
        case 'P': // DCS - Device Control String (treat as OSC)
            state = STATE_OSC;
            break;
        case '(': // SCS - G0 character set selection
            state = STATE_G0_SET;
            break;
        case ')': // SCS - G1 character set selection
            state = STATE_G1_SET;
            break;
        default:
            // not a valid escape sequence, should we print an error?
            break;
        }
        break;

    case STATE_CS: // in Control Sequence
        debug[debug_index++] = ch;
        if (ch == CHR_ESC)
        {
            state = STATE_ESCAPE;
            break; // reset to escape state
        }
        else if (ch == '?') // DEC private mode
        {
            state = STATE_DEC;
        }
        else if (ch == '!') // TMC
        {
            state = STATE_TMC;
        }
        else if (ch >= '0' && ch <= '9')
        {
            parameters[p_index] *= 10; // accumulate digits
            parameters[p_index] += ch - '0';
        }
        else if (ch == ';') // delimiter
        {
            if (p_index < sizeof(parameters) - 1)
            {
                p_index++;
            }
        }
        else // final character in control sequence
        {
            state = STATE_NORMAL; // reset state after processing the control sequence
            switch (ch)
            {
            case 'A': // CUU – Cursor Up
                row = MAX(0, row - parameters[0]);
                break;
            case 'B': // CUD – Cursor Down
                row = MIN(row + parameters[0], max_row);
                break;
            case 'C': // CUF – Cursor Forward
                column = MIN(column + parameters[0], max_col);
                break;
            case 'D': // CUB - Cursor Backward
                column = MAX(0, column - parameters[0]);
                break;
            case 'E': // CNL – CursorNext Line
                if (parameters[0] == 0)
                {
                    parameters[0] = 1; // default to 1 if not specified
                }
                row = MIN(row + parameters[0], max_row);
                column = 0;
                break;
            case 'F': // CPL – Cursor Previous Line
                if (parameters[0] == 0)
                {
                    parameters[0] = 1; // default to 1 if not specified
                }
                row = MAX(0, row - parameters[0]);
                column = 0;
                break;
            case 'G': // CHA - Cursor Horizontal Absolute
                if (parameters[0] == 0)
                {
                    parameters[0] = 1; // default to 1 if not specified
                }
                column = MIN(parameters[0] - 1, max_col);
                column = MAX(0, column);
                break;

            case 'H': // CUP – Cursor Position
            case 'f': // HVP – Horizontal and Vertical Position
                if (parameters[0] == 0)
                {
                    parameters[0] = 1; // default to 1 if not specified
                }
                if (parameters[1] == 0)
                {
                    parameters[1] = 1; // default to 1 if not specified
                }
                row = MIN(parameters[0] - 1, max_row);
                column = MIN(parameters[1] - 1, max_col);
                break;
            case 'J': // ED – Erase In Display
                if (parameters[0] == 0)
                {
                    // Erase from cursor to end of screen
                    lcd_erase_line(row, column, max_col);
                    for (uint8_t r = row + 1; r <= max_row; r++)
                    {
                        lcd_erase_line(r, 0, max_col);
                    }
                }
                else if (parameters[0] == 1)
                {
                    // Erase from start of screen to cursor
                    for (uint8_t r = 0; r < row; r++)
                    {
                        lcd_erase_line(r, 0, max_col);
                    }
                    lcd_erase_line(row, 0, column);
                }
                else if (parameters[0] == 2) // clear entire screen
                {
                    lcd_clear_screen();
                }
                break;
            case 'K': // EL – Erase In Line
                if (parameters[0] == 0)
                {
                    // Erase from cursor to end of line
                    lcd_erase_line(row, column, max_col);
                }
                else if (parameters[0] == 1)
                {
                    // Erase from start of line to cursor
                    lcd_erase_line(row, 0, column);
                }
                else if (parameters[0] == 2) // clear entire line
                {
                    lcd_erase_line(row, 0, max_col);
                }
                break;
            case 'S': // SU - Scroll Up
                if (parameters[0] == 0)
                {
                    parameters[0] = 1; // default to 1 if not specified
                }
                while (parameters[0]-- > 0)
                {
                    lcd_scroll_up();
                }
                break;
            case 'T': // SD - Scroll Down
                if (parameters[0] == 0)
                {
                    parameters[0] = 1; // default to 1 if not specified
                }
                while (parameters[0]-- > 0)
                {
                    lcd_scroll_down();
                }
                break;
            case 'c': // DA - Device Attributes
                report("\033[?1;c");
                break;
            case 'd': // VPA - Vertical Position Absolute
                if (parameters[0] == 0)
                {
                    parameters[0] = 1; // default to 1 if not specified
                }
                row = MIN(parameters[0] - 1, max_row);
                break;
            case 'e': // VPR - Vertical Position Relative
                if (parameters[0] == 0)
                {
                    parameters[0] = 1; // default to 1 if not specified
                }
                row = MIN(row + parameters[0], max_row);
                break;
            case 'g': // TBC – Tabulation Clear
                if (parameters[0] == 3)
                {
                    // Clear all tab stops
                    memset(tab_stops, 0, sizeof(tab_stops));
                }
                else if (parameters[0] == 0)
                {
                    // Clear tab stop at current column
                    if (column < sizeof(tab_stops))
                    {
                        tab_stops[column] = false;
                    }
                }
                break;
            case 'l': // RM – Reset Mode
            case 'h': // SM – Set Mode
                break;
            case 'm': // SGR – Select Graphic Rendition
                for (uint8_t i = 0; i <= p_index; i++)
                {
                    if (parameters[i] == 0) // attributes off
                    {
                        lcd_set_foreground(FOREGROUND);
                        lcd_set_background(BACKGROUND);
                        lcd_set_underscore(false);
                        lcd_set_reverse(false);
                        lcd_set_bold(false);
                    }
                    else if (parameters[i] == 1) // bold
                    {
                        lcd_set_bold(true);
                    }
                    else if (parameters[i] == 2) // dim
                    {
                        lcd_set_foreground(DIM);
                    }
                    // No support for italic (3)
                    else if (parameters[i] == 4) // underline
                    {
                        lcd_set_underscore(true);
                    }
                    // No support for blink (5, 6)
                    else if (parameters[i] == 7) // negative (reverse) image
                    {
                        lcd_set_reverse(true);
                    }
                    else if (parameters[i] == 22) // normal intensity/weight
                    {
                        lcd_set_foreground(FOREGROUND);
                        lcd_set_bold(false);
                    }
                    else if (parameters[i] == 24) // not underlined
                    {
                        lcd_set_underscore(false);
                    }
                    else if (parameters[i] == 27) // positive image
                    {
                        lcd_set_reverse(false);
                    }
                    else if (parameters[i] >= 30 && parameters[i] <= 37) // foreground colour
                    {
                        lcd_set_foreground(palette[parameters[i] - 30]);
                    }
                    else if (parameters[i] == 38 && i + 4 <= p_index && parameters[i + 1] == 2) // foreground truecolor
                    {
                        uint8_t r = parameters[i + 2];
                        uint8_t g = parameters[i + 3];
                        uint8_t b = parameters[i + 4];
                        uint16_t colour = RGB(r, g, b);
                        lcd_set_foreground(colour);
                        i += 4; // Skip the next four parameters (2, r, g, b)
                    }
                    else if (parameters[i] == 38 && i + 2 <= p_index && parameters[i + 1] == 5) // foreground 256-colour
                    {
                        uint8_t colour = parameters[i + 2];
                        lcd_set_foreground(xterm_palette[colour]);
                        i += 2; // Skip the next two parameters (5 and colour)
                    }
                    else if (parameters[i] == 39) // default foreground colour
                    {
                        lcd_set_foreground(FOREGROUND);
                    }
                    else if (parameters[i] >= 40 && parameters[i] <= 47) // background colour
                    {
                        lcd_set_background(palette[parameters[i] - 40]);
                    }
                    else if (parameters[i] == 48 && i + 4 <= p_index && parameters[i + 1] == 2) // background truecolor
                    {
                        uint8_t r = parameters[i + 2];
                        uint8_t g = parameters[i + 3];
                        uint8_t b = parameters[i + 4];
                        uint16_t colour = RGB(r, g, b);
                        lcd_set_background(colour);
                        i += 4; // Skip the next four parameters (2, r, g, b)
                    }
                    else if (parameters[i] == 48 && i + 2 <= p_index && parameters[i + 1] == 5) // background 256-colour
                    {
                        uint8_t colour = parameters[i + 2];
                        lcd_set_background(xterm_palette[colour]);
                        i += 2; // Skip the next two parameters (5 and colour)
                    }
                    else if (parameters[i] == 49) // default background colour
                    {
                        lcd_set_background(BACKGROUND);
                    }
                    else if (parameters[i] >= 90 && parameters[i] <= 97) // bright foreground colour
                    {
                        uint8_t index = parameters[i] - 90;
                        if (index < 8) // ensure index is within bounds
                        {
                            lcd_set_foreground(bright_palette[index]);
                        }
                    }
                    else if (parameters[i] >= 100 && parameters[i] <= 107) // bright background colour
                    {
                        uint8_t index = parameters[i] - 100;
                        if (index < 8) // ensure index is within bounds
                        {
                            lcd_set_background(bright_palette[index]);
                        }
                    }
                }
                break;
            case 'n':                   // Device Status Report
                if (parameters[0] == 5) // DSR - Device Status Report
                {
                    report("\033[0n");
                }
                else if (parameters[0] == 6) // DSR - Device Status Report
                {
                    char buf[16];
                    snprintf(buf, sizeof(buf), "\033[%d;%dR", row + 1, column + 1);
                    report(buf);
                }
                break;
            case CHR_CAN:                      // cancel the current escape sequence
            case CHR_SUB:                      // same as CAN
                lcd_putc(column++, row, 0x02); // print a error character
                break;
            case 'q': // DECLL – Load LEDS (DEC Private)
                for (uint8_t i = 0; i <= p_index; i++)
                {
                    if (parameters[i] == 0) // turn off all LEDs
                    {
                        leds = 0; // reset LED state
                    }
                    else if (parameters[i] > 0 && parameters[i] <= 8)
                    {
                        leds |= (1 << (parameters[i] - 1));
                    }
                }
                update_leds(leds); // update the LEDs
                break;
            case 'r': // DECSTBM – Set Top and Bottom Margins
                if (parameters[0] == 0)
                {
                    parameters[0] = 1; // default to 1 if not specified
                }
                if (parameters[1] == 0)
                {
                    parameters[1] = 1; // default to 1 if not specified
                }
                uint8_t top_row = MIN(parameters[0] - 1, max_row);
                uint8_t bottom_row = MIN(parameters[1] - 1, max_row);
                if (bottom_row > top_row)
                {
                    lcd_define_scrolling(top_row, max_row - bottom_row);
                }
                else
                {
                    lcd_scroll_reset();
                }
                row = top_row;
                column = 0;
                break;
            case 's': // DECSC – Save Cursor (ANSI)
                save_column = column;
                save_row = row;
                break;
            case 't': // - Lines per page
                // Not supported, ignore
                break;
            case 'u': // DECRC – Restore Cursor (ANSI)
                column = save_column;
                row = save_row;
                break;
            default:
                lcd_putc(column++, row, 0x02); // print a error character
                break;                         // ignore unknown sequences
            }
        }
        break;

    case STATE_TMC:
        switch (ch)
        {
        case 'p': // Soft reset
            reset_terminal();

            break;
        }
        state = STATE_NORMAL;
        break;

    case STATE_DEC: // in DEC private mode sequence
        debug[debug_index++] = ch;
        if (ch == CHR_ESC)
        {
            state = STATE_ESCAPE;
            break; // reset to escape state
        }
        else if (ch >= '0' && ch <= '9')
        {
            parameters[p_index] *= 10; // accumulate digits
            parameters[p_index] += ch - '0';
        }
        else if (ch == ';') // delimiter
        {
            if (p_index < sizeof(parameters) - 1)
            {
                p_index++;
            }
        }
        else // final character in DEC private mode sequence
        {
            state = STATE_NORMAL; // reset state after processing
            switch (ch)
            {
            case 'h':                    // DECSET - DEC Private Mode Set
                if (parameters[0] == 25) // DECTCEM - Text Cursor Enable Mode
                {
                    lcd_enable_cursor(true);
                    lcd_draw_cursor();
                }
                else if (parameters[0] == 4264)
                {
                    // set 64 column mode
                    lcd_set_font(&font_5x10);
                }
                break;
            case 'l':                    // DECRST - DEC Private Mode Reset
                if (parameters[0] == 25) // DECTCEM - Text Cursor Enable Mode
                {
                    lcd_enable_cursor(false);
                    lcd_erase_cursor(); // immediately hide cursor
                }
                else if (parameters[0] == 4264)
                {
                    // set 64 column mode
                    lcd_set_font(&font_8x10);
                }
                break;
            case 'm':
                // Ignore for now
                break;
            default:
                lcd_putc(column++, row, 0x01); // print a error character
                break;                         // ignore unknown DEC private mode sequences
            }
        }
        break;

    case STATE_G0_SET:        // Setting G0 character set
        state = STATE_NORMAL; // return to normal state after processing
        switch (ch)
        {
        case 'A': // UK character set
            set_g0_charset(CHARSET_UK);
            break;
        case 'B': // ASCII character set
            set_g0_charset(CHARSET_ASCII);
            break;
        case '0': // DEC Special Character Set
            set_g0_charset(CHARSET_DEC);
            break;
        default:
            // Unknown character set, ignore
            break;
        }
        break;

    case STATE_G1_SET:        // Setting G1 character set
        state = STATE_NORMAL; // return to normal state after processing
        switch (ch)
        {
        case 'A': // UK character set
            set_g1_charset(CHARSET_UK);
            break;
        case 'B': // ASCII character set
            set_g1_charset(CHARSET_ASCII);
            break;
        case '0': // DEC Special Character Set
            set_g1_charset(CHARSET_DEC);
            break;
        default:
            // Unknown character set, ignore
            break;
        }
        break;

    case STATE_OSC: // in Operating System Command
        if (ch == CHR_ESC)
        {
            state = STATE_OSC_ESC;
        }
        else if (ch == '\007' || ch == 0x9C)
        {
            state = STATE_NORMAL; // reset state after processing the OSC sequence
        }
        break;

    case STATE_OSC_ESC: // in OSC escape sequence
        state = ch == '\\' ? STATE_NORMAL : STATE_OSC;
        break;

    case STATE_NORMAL:
    default:
        // Normal/default state, process characters directly
        switch (ch)
        {
        case CHR_BS:
            column = MAX(0, column - 1); // move cursor back one space (but not before the start of the line)
            break;
        case CHR_BEL:
            ring_bell(); // ring the bell
            break;
        case CHR_HT:
            column = MIN(((column + 8) & ~7), lcd_get_columns() - 1); // move cursor to next tabstop (but not beyond the end of the line)
            break;
        case CHR_LF:
        case CHR_VT:
        case CHR_FF:
            row++; // move cursor down one line
            break;
        case CHR_CR:
            column = 0; // move cursor to the start of the line
            break;
        case CHR_SO: // Shift Out - select G1 character set
            set_charset(G1_CHARSET);
            break;
        case CHR_SI: // Shift In - select G0 character set
            set_charset(G0_CHARSET);
            break;
        case CHR_ESC:
            state = STATE_ESCAPE;
            break;
        default:
            if (ch >= 0x20 && ch < 0x7F) // printable characters
            {
                // Translate character based on active character set
                if (get_charset() == CHARSET_UK && ch == '#')
                {
                    // Replace '#' with the pound sign in UK character set
                    ch = 0x1E;
                }
                else if (get_charset() == CHARSET_DEC && ch >= 0x5F && ch <= 0x7E)
                {
                    // Maps characters 0x5F - 0x7E to DEC Special Character Set
                    ch -= 0x5F;
                }

                lcd_putc(column++, row, ch);
            }
            break;
        }
        break;
    }

    // Handle wrapping and scrolling
    if (column > max_col) // wrap around at end of the line
    {
        column = 0;
        row++;
    }

    if (row > max_row) // scroll at bottom of the screen
    {
        while (row > max_row) // scroll until y is within bounds
        {
            lcd_scroll_up(); // scroll up to make space at the bottom
            row--;
        }
    }

    // Update cursor position
    lcd_move_cursor(column, row);
    lcd_draw_cursor(); // draw the cursor at the new position
}

//
//  Display Callback Setters
//

void display_set_led_callback(led_callback_t callback)
{
    display_led_callback = callback;
}

void display_set_bell_callback(bell_callback_t callback)
{
    display_bell_callback = callback;
}

void display_set_report_callback(report_callback_t callback)
{
    display_report_callback = callback;
}

//
//  Display Initialization
//

void display_init()
{
    // Make sure the LCD is initialized
    lcd_init();

    // Set tab stops every 8 columns by default
    for (int i = 3; i < 64; i += 8)
    {
        tab_stops[i] = true;
    }
}