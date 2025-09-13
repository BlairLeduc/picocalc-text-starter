#pragma once

#include "pico/stdlib.h"
#include "font.h"

// Processing ANSI escape sequences is a small state machine. These
// are the states.
#define STATE_NORMAL    (0)             // normal state
#define STATE_ESCAPE    (1)             // escape character received
#define STATE_CS        (2)             // control sequence introducer (CSI) received
#define STATE_DEC       (3)             // DEC private mode sequence (?)
#define STATE_G0_SET    (4)             // setting G0 character set ESC(
#define STATE_G1_SET    (5)             // setting G1 character set ESC)
#define STATE_OSC       (6)             // Operating System Command (OSC)
#define STATE_OSC_ESC   (7)             // Operating System Command (OSC) ESC
#define STATE_TMC       (8)             // Terminal Management Control (TMC)

// Control characters
#define CHR_BEL         (0x07)          // Bell
#define CHR_BS          (0x08)          // Backspace
#define CHR_HT          (0x09)          // Horizontal Tab
#define CHR_LF          (0x0A)          // Line Feed
#define CHR_VT          (0x0B)          // Vertical Tab
#define CHR_FF          (0x0C)          // Form Feed
#define CHR_CR          (0x0D)          // Carriage Return
#define CHR_SO          (0x0E)          // Shift Out (select G1 character set)
#define CHR_SI          (0x0F)          // Shift In (select G0 character set)
#define CHR_CAN         (0x18)          // Cancel
#define CHR_SUB         (0x1A)          // Substitute
#define CHR_ESC         (0x1B)          // Escape

// Character set definitions
#define CHARSET_UK      (0)             // UK character set
#define CHARSET_ASCII   (1)             // ASCII character set
#define CHARSET_DEC     (2)             // DEC Special Character Set (line drawing)

#define G0_CHARSET      (0)             // G0 character set
#define G1_CHARSET      (1)             // G0 and G1 character sets

// Defaults
#define WHITE_PHOSPHOR  RGB(216, 240, 255)  // white phosphor
#define GREEN_PHOSPHOR  RGB(51, 255, 102)   // green phosphor
#define AMBER_PHOSPHOR  RGB(255, 255, 51)   // amber phosphor
#define FOREGROUND      RGB(255, 255, 255)  // default foreground colour
#define BACKGROUND      RGB(0, 0, 0)        // default background colour
#define BRIGHT          RGB(255, 255, 255)  // white
#define DIM             RGB(128, 128, 128)  // dim grey

// Notify when the led state changes
// LSB = L1
typedef void (*led_callback_t)(uint8_t);

// Notify then the bell character is received
typedef void (*bell_callback_t)(void);

// Notify when a terminal report is requested
typedef void (*report_callback_t)(const char *);

// Function prototypes
void display_init(void);
void display_set_led_callback(led_callback_t callback);
void display_set_bell_callback(bell_callback_t callback);
void display_set_report_callback(report_callback_t callback);
bool display_emit_available(void);
void display_emit(char c);