#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "pico/bootrom.h"
#include "pico/float.h"
#include "pico/util/datetime.h"
#include "pico/time.h"

#include "drivers/southbridge.h"
#include "drivers/audio.h"
#include "drivers/sdcard.h"
#include "songs.h"
#include "tests.h"
#include "commands.h"

volatile bool user_interrupt = false;

// Command table - map of command names to functions
static const command_t commands[] = {
    {"backlight", backlight, "Show the backlight levels"},
    {"battery", battery, "Show the battery level"},
    {"beep", beep, "Play a simple beep sound"},
    {"box", box, "Draw a box on the screen"},
    {"bye", bye, "Reboot into BOOTSEL mode"},
    {"cls", clearscreen, "Clear the screen"},
    {"play", play, "Play a song"},
    {"dir", sd_list, "List files on SD card"},
    {"mount", sd_mount_cmd, "Mount SD card"},
    {"more", sd_read_file, "Page through a file"},
    {"sd_status", sd_status, "Show SD card status"},
    {"songs", show_song_library, "Show song library"},
    {"test", test, "Run a test"},
    {"tests", show_test_library, "Show test library"},
    {"help", help, "Show this help message"},
    {NULL, NULL, NULL} // Sentinel to mark end of array
};

// Extended song command that takes a parameter
static void play_named_song(const char *song_name)
{
    const audio_song_t *song = find_song(song_name);
    if (!song)
    {
        printf("Song '%s' not found.\n", song_name);
        printf("Use 'songs' command to see available\nsongs.\n");
        return;
    }

    printf("\nNow playing:\n%s\n\n", song->description);
    printf("Press BREAK key to stop...\n");

    // Reset user interrupt flag
    user_interrupt = false;

    audio_play_song_blocking(song);

    if (user_interrupt)
    {
        printf("\nPlayback interrupted by user.\n");
    }
    else
    {
        printf("\nSong finished!\n");
    }
}

static void run_named_test(const char * test_name)
{
    const test_t *test = find_test(test_name);
    if (!test)
    {
        printf("Test '%s' not found.\n", test_name);
        printf("Use 'tests' command to see available\ntests.\n");
        return;
    }

    printf("Running test: %s\n", test->name);
    printf("Press BREAK key to stop...\n");

    // Reset user interrupt flag
    user_interrupt = false; 
    test->function(); // Call the test function

    if (user_interrupt)
    {
        printf("\nTest interrupted by user.\n");
    }
    else
    {
        printf("\nTest finished!\n");
    }

}

void run_command(const char *command)
{
    bool command_found = false; // Flag to check if command was found

    // Make a copy of the command string for parsing
    char cmd_copy[256];
    strncpy(cmd_copy, command, sizeof(cmd_copy) - 1);
    cmd_copy[sizeof(cmd_copy) - 1] = '\0';

    // Parse command and arguments
    char *cmd_name = strtok(cmd_copy, " ");
    char *cmd_arg = strtok(NULL, " ");

    if (!cmd_name)
    {
        return; // Empty command
    }

    // Search for command in the table
    for (int i = 0; commands[i].name != NULL; i++)
    {
        if (strcmp(cmd_name, commands[i].name) == 0)
        {
            // Special handling for song commands with arguments
            if (strcmp(cmd_name, "play") == 0 && cmd_arg != NULL)
            {
                play_named_song(cmd_arg);
            }
            else if (strcmp(cmd_name, "more") == 0 && cmd_arg != NULL)
            {
                sd_read_filename(cmd_arg);
            }
            else if (strcmp(cmd_name, "test") == 0 && cmd_arg != NULL)
            {
                run_named_test(cmd_arg);
            }
            else
            {
                commands[i].function(); // Call the command function
            }
            command_found = true; // Command found and executed
            break;                // Exit the loop
        }
    }

    if (!command_found)
    {
        printf("%s ?\nType 'help' for a list of commands.\n", cmd_name);
    }
    user_interrupt = false;
}

void help()
{
    printf("Available commands:\n");
    for (int i = 0; commands[i].name != NULL; i++)
    {
        printf("  %s - %s\n", commands[i].name, commands[i].description);
    }
}

void backlight()
{
    uint8_t lcd_backlight = sb_read_lcd_backlight();
    uint8_t keyboard_backlight = sb_read_keyboard_backlight();

    printf("LCD BackLight: %.0f%%\n", lcd_backlight / 2.55);           // Convert to percentage
    printf("Keyboard BackLight: %.0f%%\n", keyboard_backlight / 2.55); // Convert to percentage
}

void battery()
{
    int raw_level = sb_read_battery();
    int battery_level = raw_level & 0x7F;    // Mask out the charging bit
    bool charging = (raw_level & 0x80) != 0; // Check if charging

    printf("\033[?25l\033(0");
    if (charging)
    {
        printf("\033[93m");
    }
    else
    {
        printf("\033[97m");
    }
    printf("lqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqk\n");
    printf("x ");
    if (battery_level < 10)
    {
        printf("\033[101m"); // Set background colour to red for critical battery
    }
    else if (battery_level < 30)
    {
        printf("\033[103m"); // Set background colour to yellow for low battery
    }
    else
    {
        printf("\033[102m"); // Set background colour to green for sufficient battery
    }
    for (int i = 0; i < battery_level / 3; i++)
    {
        printf(" "); // Print a coloured space for each 3% of battery
    }
    printf("\033[37;40m"); // Reset colour
    for (int i = battery_level / 3; i < 33; i++)
    {
        printf("a"); // Fill the rest of the bar with fuzz
    }
    if (charging)
    {
        printf("\033[93m");
    }
    else
    {
        printf("\033[97m");
    }
    printf(" x\n");
    printf("mqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqj\n");

    // Switch back to ASCII, turn on cursor and reset character attributes
    printf("\033(B\033[?25h\033[m\n");

    if (charging)
    {
        printf("Battery level: %d%% (charging)\n", battery_level);
    }
    else
    {
        printf("Battery level: %d%%\n", battery_level);
    }
}

void beep()
{
    printf("Playing beep...\n");
    audio_play_sound_blocking(HIGH_BEEP, HIGH_BEEP, NOTE_QUARTER);
    printf("Beep complete.\n");
}

void box()
{
    printf("A box using the DEC Special Character\nSet:\n\n");
    printf("\033[93m");  // Set foreground colour to bright yellow
    printf("\033[?25l"); // Hide cursor

    // Switch to DEC Special Character Set and draw a box
    // Visual representation of what will be drawn:
    //   ┌──────┬──────┐
    //   │      │      │
    //   ├──────┼──────┤
    //   │      │      │
    //   └──────┴──────┘
    //
    // DEC Special Character mappings:
    // l = ┌ (top-left)     q = ─ (horizontal)   k = ┐ (top-right)
    // x = │ (vertical)     w = ┬ (top-tee)
    // t = ├ (left-tee)     n = ┼ (cross)        u = ┤ (right-tee)
    // m = └ (bottom-left)  v = ┴ (bottom-tee)   j = ┘ (bottom-right)

    printf("\033(0");          // Select DEC Special Character Set for G0
    printf("lqqqqqwqqqqqk\n"); // ┌──────┬──────┐
    printf("x     x     x\n"); // │      │      │
    printf("tqqqqqnqqqqqu\n"); // ├──────┼──────┤
    printf("x     x     x\n"); // │      │      │
    printf("mqqqqqvqqqqqj\n"); // └──────┴──────┘

    // Switch back to ASCII
    printf("\033(B\033[?25h"); // Select ASCII for G0 and show cursor
    printf("\033[0m");         // Reset colors
    printf("\n\nSee source code for the box drawing\ncharacters.\n");
}

void bye()
{
    printf("Exiting...\n");
    rom_reset_usb_boot(0, 0);
}

void clearscreen()
{
    printf("\033[2J\033[H"); // ANSI escape code to clear the screen
}

void play()
{
    printf("Error: No song specified.\n");
    printf("Usage: play <name>\n");
    printf("Use 'songs' command to see available\nsongs.\n");
}

void test()
{
    printf("Error: No test specified.\n");
    printf("Usage: test <name>\n");
    printf("Use 'tests' command to see available\ntests.\n");
}

//
// SD Card Commands
//

void sd_status()
{
    printf("SD Card Status:\n");
    
    if (!sd_card_present()) {
        printf("  No SD card detected\n");
        return;
    }
    
    printf("  Card detected: Yes\n");
    
    sd_error_t result = sd_init();
    if (result != SD_OK) {
        printf("  Initialization: Failed (%s)\n", sd_error_string(result));
        return;
    }
    
    printf("  Initialization: OK\n");
    
    if (sd_is_mounted()) {
        printf("  File system: Mounted\n");
        fs_type_t fs_type = sd_get_fs_type();
        const char* fs_name = "Unknown";
        switch (fs_type) {
            case FS_TYPE_FAT12: fs_name = "FAT12"; break;
            case FS_TYPE_FAT16: fs_name = "FAT16"; break;
            case FS_TYPE_FAT32: fs_name = "FAT32"; break;
            default: break;
        }
        printf("  File system type: %s\n", fs_name);
        
        uint32_t total_space = sd_get_total_space();
        printf("  Total space: %lu bytes\n", total_space);
    } else {
        printf("  File system: Not mounted\n");
    }
}

void sd_mount_cmd()
{
    printf("Mounting SD card...\n");
    
    if (!sd_card_present()) {
        printf("Error: No SD card detected\n");
        return;
    }
    
    sd_error_t result = sd_mount();
    if (result == SD_OK) {
        printf("SD card mounted successfully\n");
        fs_type_t fs_type = sd_get_fs_type();
        const char* fs_name = "Unknown";
        switch (fs_type) {
            case FS_TYPE_FAT12: fs_name = "FAT12"; break;
            case FS_TYPE_FAT16: fs_name = "FAT16"; break;
            case FS_TYPE_FAT32: fs_name = "FAT32"; break;
            default: break;
        }
        printf("File system: %s\n", fs_name);
    } else {
        printf("Error: Failed to mount SD card (%s)\n", sd_error_string(result));
    }
}

void sd_list()
{
    if (!sd_is_mounted()) {
        printf("Error: SD card not mounted\n");
        printf("Use 'mount' command first\n");
        return;
    }
    
    // Simple directory listing for root directory
    sd_error_t result = sd_list_root_directory();
    if (result != SD_OK) {
        printf("Error listing directory: %s\n", sd_error_string(result));
    }
}

void sd_read_file()
{
    printf("Error: No filename specified.\n");
    printf("Usage: sd_read <filename>\n");
    printf("Example: sd_read readme.txt\n");
}

void sd_read_filename(const char *filename)
{
    if (filename == NULL || strlen(filename) == 0) {
        printf("Error: No filename specified.\n");
        printf("Usage: sd_read <filename>\n");
        printf("Example: sd_read readme.txt\n");
        return;
    }
    // For demonstration, try to read a common filename
    if (!sd_is_mounted()) {
        printf("\nNote: SD card not mounted\n");
        printf("Use 'mount' command first\n");
        return;
    }
    
    sd_file_t file;
    sd_error_t result = sd_file_open(&file, filename);
    
    if (result != SD_OK) {
        printf("Error: %s\n", sd_error_string(result));
        return;
    }
    
    // Read and display the file content with pagination
    char buffer[1024];  // Read in larger chunks
    size_t bytes_read;
    uint32_t total_bytes_read = 0;
    int line_count = 0;
    bool user_quit = false;
    
    while (!user_quit && total_bytes_read < sd_file_size(&file)) {
        result = sd_file_read(&file, buffer, sizeof(buffer) - 1, &bytes_read);
        if (result != SD_OK || bytes_read == 0) {
            if (result != SD_OK) {
                printf("Error reading file: %s\n", sd_error_string(result));
            }
            break;
        }
        
        buffer[bytes_read] = '\0';  // Null terminate
        total_bytes_read += bytes_read;
        
        // Display the content line by line
        char *line_start = buffer;
        char *line_end;
        
        while ((line_end = strchr(line_start, '\n')) != NULL || line_start < buffer + bytes_read) {
            if (line_end == NULL) {
                // Last line without newline
                printf("%s", line_start);
                break;
            }
            
            // Print the line (including newline)
            *line_end = '\0';
            printf("%s\n", line_start);
            line_count++;
            
            // Check if we need to pause
            if (line_count >= 31) {
                printf("\nMore?");
                char ch = getchar();
                if (ch == 'q' || ch == 'Q') {
                    user_quit = true;
                    printf("\n");
                    break;
                }
                printf("\n");
                line_count = 0;
            }
            
            line_start = line_end + 1;
        }
        
        if (user_quit) break;
    }
    
    sd_file_close(&file);
}

void sd_debug_cmd()
{
    printf("SD Card Debug Test\n");
    printf("==================\n");
    
    sd_error_t result = sd_debug_init();
    if (result == SD_OK) {
        printf("Debug initialization successful!\n");
        printf("Basic SPI communication is working.\n");
    } else {
        printf("Debug initialization failed: %s\n", sd_error_string(result));
        printf("Check your wiring and card insertion.\n");
    }
}

// SD dump sector command
void sd_dump_sector(void) {
    printf("Usage: This function needs to be called with a sector number\n");
    printf("Dumping sector 0 (boot sector) as example:\n");
    sd_debug_dump_sector(0);
}
