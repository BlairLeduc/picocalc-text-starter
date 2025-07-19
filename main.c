#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "drivers/picocalc.h"
#include "drivers/keyboard.h"
#include "drivers/onboard_led.h"

#include "commands.h"

void set_onboard_led(uint8_t led)
{
    led_set(led & 0x01);
}

void readline(char *buffer, size_t size)
{
    size_t index = 0;
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
}

int main()
{
    char buffer[40];
    uint8_t index = 0;

    // Initialize the LED driver and set the LED callback
    // If the LED driver fails to initialize, we can still run the text starter
    // without LED support, so we pass NULL to picocalc_init.
    int led_init_result = led_init();

    stdio_init_all();
    picocalc_init(led_init_result == 0 ? set_onboard_led : NULL);

    printf("\033c\033[1m Hello from the PicoCalc Text Starter!\033[0m\n\n");
    printf("      Contributed to the community\n");
    printf("            by Blair Leduc.\n\n");
    printf("Type \033[4mhelp\033[0m for a list of commands.\n\n");

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

        run_command(buffer); // Call the command handler

        printf("\033[q\nReady.\n"); // Turn off the LED and prompt for input again
    }
}
