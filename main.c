#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "drivers/picocalc.h"
#include "drivers/display.h"
#include "drivers/keyboard.h"
#include "drivers/onboard_led.h"
#include "drivers/lcd.h"
#include "drivers/ds3231.h"

#include "commands.h"
#include "wifi.h"
#include "tiles.h"
#include "gfx_core.h"

bool power_off_requested = false;

// Command history
#define HISTORY_SIZE 10
#define HISTORY_BUFFER_SIZE 40
static char history[HISTORY_SIZE][HISTORY_BUFFER_SIZE];
static int history_count = 0;

void set_onboard_led(uint8_t led)
{
    led_set(led & 0x01);
}

#include <ctype.h>

void str_to_lower(char *s) {
    while (*s) {
        *s = tolower((unsigned char)*s);
        s++;
    }
}

// Safe string copy that always null-terminates
static inline void safe_strcpy(char *dst, const char *src, size_t dst_size) {
    if (dst_size == 0) return;
    size_t i;
    for (i = 0; i < dst_size - 1 && src[i] != '\0'; i++) {
        dst[i] = src[i];
    }
    dst[i] = '\0';
}

void readline(char *buffer, size_t size)
{
    size_t index = 0;
    int history_pos = -1; // -1 means "not navigating history"
    char temp_buffer[HISTORY_BUFFER_SIZE] = {0}; // Temporary buffer for current command

    while (true)
    {
        char ch = getchar();
        if (ch == 0x04) // Ctrl+D to debug
        {
            printf("Entering debug mode...\n");
            __breakpoint();
        }
        else if (ch == '\n' || ch == '\r')
        {
            printf("\n");
            break; // End of line
        }
        else if (ch == KEY_UP) // UP arrow - navigate history upward
        {
            if (history_count == 0) continue;

            // Save current command the first time
            if (history_pos == -1)
            {
                safe_strcpy(temp_buffer, buffer, HISTORY_BUFFER_SIZE);
                history_pos = history_count;
            }

            if (history_pos > 0)
            {
                history_pos--;

                // Clear current line
                while (index > 0)
                {
                    printf("\b \b");
                    index--;
                }

                // Copy and display command from history
                safe_strcpy(buffer, history[history_pos], size);
                index = strlen(buffer);
                printf("%s", buffer);
            }
        }
        else if (ch == KEY_DOWN) // DOWN arrow - navigate history downward
        {
            if (history_pos == -1) continue;

            // Clear current line
            while (index > 0)
            {
                printf("\b \b");
                index--;
            }

            if (history_pos < history_count - 1)
            {
                history_pos++;
                safe_strcpy(buffer, history[history_pos], size);
            }
            else
            {
                // Return to what was being typed
                history_pos = -1;
                safe_strcpy(buffer, temp_buffer, size);
            }

            index = strlen(buffer);
            printf("%s", buffer);
        }
        else if ((ch == 0x08 || ch == 0x7F) && index > 0) // Backspace or Delete
        {
            index--;
            buffer[index] = '\0'; // Remove last character
            printf("\b \b"); // Erase the last character
        }
        else if (ch >= 0x20 && ch < 0x7F && index < size - 1) // Printable characters
        {
            buffer[index++] = ch;
            putchar(ch);
        }
    }
    buffer[index] = '\0'; // Null-terminate the string

    // Add command to history if not empty and different from last
    if (index > 0 && (history_count == 0 || strcmp(buffer, history[history_count - 1]) != 0))
    {
        // If history is full, shift all elements back
        if (history_count >= HISTORY_SIZE)
        {
            for (int i = 0; i < HISTORY_SIZE - 1; i++)
            {
                safe_strcpy(history[i], history[i + 1], HISTORY_BUFFER_SIZE);
            }
            history_count = HISTORY_SIZE - 1;
        }

        // Add new command
        safe_strcpy(history[history_count], buffer, HISTORY_BUFFER_SIZE);
        history_count++;
    }
}


int main()
{
    char buffer[40];

    // Initialize the LED driver and set the LED callback
    // If the LED driver fails to initialize, we can still run the text starter
    // without LED support, so we pass NULL to picocalc_init.
    int led_init_result = led_init();

    stdio_init_all();
    picocalc_init();
    if (led_init_result == 0) {
        display_set_led_callback(set_onboard_led);
    }

    // Initialize Core 1 for graphics processing AFTER hardware is ready
    gfx_core_init();


    printf("\033c\033[1m\n Hello from the PicoCalc Text Starter!\033[0m\n\n");
    printf("      Contributed to the community\n");
    printf("            by Blair Leduc.\n\n");
    printf("Type \033[4mhelp\033[0m for a list of commands.\n");
    printf("Vers 1.2 Jobond \n\n");

    // Inizializza il DS3231 RTC
    if (ds3231_init()) {
        printf("DS3231 RTC ready on I2C1 (GP6/GP7)\n\n");
    } else {
        printf("Warning: DS3231 RTC not detected\n\n");
    }

    //JOBOND: TEMPORARILY DISABLED
    //test_wifi();

    // A very simple REPL
    printf("\033[qReady.\n");
    while (true)
    {
        readline(buffer, sizeof(buffer));
        if (strlen(buffer) == 0)
        {
            continue; // Skip empty input
        }

        printf("\033[1q\n"); // Turn on the LED so the user knows input is being processed

        // Convert the input to lowercase for case-insensitive command matching
        str_to_lower(buffer);

        run_command(buffer); // Call the command handler

        printf("\033[q\nReady. %s\n", power_off_requested ? "(power off requested)" : ""); // Turn off the LED and prompt for input again
        power_off_requested = false;
    }
}
