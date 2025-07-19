#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <errno.h>

#include "pico/bootrom.h"
#include "pico/float.h"
#include "pico/util/datetime.h"
#include "pico/time.h"

#include "drivers/southbridge.h"
#include "drivers/audio.h"
#include "drivers/sdcard.h"
#include "drivers/fat32.h"
#include "songs.h"
#include "tests.h"
#include "commands.h"

volatile bool user_interrupt = false;
extern void readline(char *buffer, size_t size);

// Command table - map of command names to functions
static const command_t commands[] = {
    {"backlight", backlight, "Show the backlight levels"},
    {"battery", battery, "Show the battery level"},
    {"beep", beep, "Play a simple beep sound"},
    {"box", box, "Draw a box on the screen"},
    {"bye", bye, "Reboot into BOOTSEL mode"},
    {"cls", clearscreen, "Clear the screen"},
    {"cd", cd, "Change directory"},
    {"dir", dir, "List files on the SD card"},
    {"free", sd_free, "Show free space on the SD card"},
    {"mkdir", sd_mkdir, "Create a new directory"},
    {"mkfile", sd_mkfile, "Create a new file"},
    {"more", sd_more, "Page through a file"},
    {"play", play, "Play a song"},
    {"pwd", sd_pwd, "Print working directory"},
    {"rm", sd_rm, "Remove a file"},
    {"rmdir", sd_rmdir, "Remove a directory"},
    {"sdcard", sd_status, "Show SD card status"},
    {"songs", show_song_library, "Show song library"},
    {"test", test, "Run a test"},
    {"tests", show_test_library, "Show test library"},
    {"help", show_command_library, "Show this help message"},
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

static void run_named_test(const char *test_name)
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
    char *cmd_name = cmd_copy;
    char *cmd_arg = NULL;
    char *space = strchr(cmd_copy, ' ');
    if (space)
    {
        *space = '\0'; // Terminate cmd_name
        cmd_arg = space + 1;
        // Skip any leading spaces in cmd_arg
        while (*cmd_arg == ' ')
            cmd_arg++;
        if (*cmd_arg == '\0')
            cmd_arg = NULL;
    }

    if (!cmd_name || *cmd_name == '\0')
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
            else if (strcmp(cmd_name, "dir") == 0 && cmd_arg != NULL)
            {
                sd_dir_dirname(cmd_arg);
            }
            else if (strcmp(cmd_name, "cd") == 0 && cmd_arg != NULL)
            {
                cd_dirname(cmd_arg);
            }
            else if (strcmp(cmd_name, "mkfile") == 0 && cmd_arg != NULL)
            {
                sd_mkfile_filename(cmd_arg);
            }
            else if (strcmp(cmd_name, "mkdir") == 0 && cmd_arg != NULL)
            {
                sd_mkdir_filename(cmd_arg);
            }
            else if (strcmp(cmd_name, "rm") == 0 && cmd_arg != NULL)
            {
                sd_rm_filename(cmd_arg);
            }
            else if (strcmp(cmd_name, "rmdir") == 0 && cmd_arg != NULL)
            {
                sd_rmdir_dirname(cmd_arg);
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

void show_command_library()
{
    printf("\033[?25l\033[4mCommand Library\033[0m\n\n");
    for (int i = 0; commands[i].name != NULL; i++)
    {
        printf("  \033[1m%s\033[0m - %s\n", commands[i].name, commands[i].description);
    }
    printf("\n\033[?25h");
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
        printf("\033[38;5;220m");
    }
    else
    {
        printf("\033[38;5;231m");
    }
    printf("lqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqk\n");
    printf("x ");
    if (battery_level < 10)
    {
        printf("\033[38;5;196;7m"); // Set background colour to red for critical battery
    }
    else if (battery_level < 30)
    {
        printf("\033[38;5;226;7m"); // Set background colour to yellow for low battery
    }
    else
    {
        printf("\033[38;5;46;7m"); // Set background colour to green for sufficient battery
    }
    for (int i = 0; i < battery_level / 3; i++)
    {
        printf(" "); // Print a coloured space for each 3% of battery
    }
    printf("\033[0;38;5;242m"); // Reset colour
    for (int i = battery_level / 3; i < 33; i++)
    {
        printf("a"); // Fill the rest of the bar with fuzz
    }
    if (charging)
    {
        printf("\033[38;5;220m");
    }
    else
    {
        printf("\033[38;5;231m");
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
    printf("\033[38;5;208m");  // Set foreground colour to orange
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

static void get_str_size(char *buffer, uint32_t buffer_size, uint64_t bytes)
{
    const char *unit = "bytes";
    uint32_t divisor = 1;

    if (bytes >= 1000 * 1000 * 1000)
    {
        divisor = 1000 * 1000 * 1000;
        unit = "GB";
    }
    else if (bytes >= 1000 * 1000)
    {
        divisor = 1000 * 1000;
        unit = "MB";
    }
    else if (bytes >= 1000)
    {
        divisor = 1000;
        unit = "KB";
    }

    if (unit == "bytes" || unit == "KB")
    {
        snprintf(buffer, buffer_size, "%llu %s", (unsigned long long)(bytes / divisor), unit);
    }
    else
    {
        snprintf(buffer, buffer_size, "%.1f %s", ((float)bytes) / divisor, unit);
    }
}

void sd_status()
{
    if (!sd_card_present())
    {
        printf("SD card not inserted\n");
        return;
    }

    sd_error_t mount_status = fat32_get_status();
    if (mount_status != FAT32_OK)
    {
        printf("SD card inserted, but unreadable.\n");
        printf("Error: %s\n", fat32_error_string(mount_status));
        return;
    }

    uint64_t total_space;
    sd_error_t result = fat32_get_total_space(&total_space);
    if (result != SD_OK)
    {
        printf("SD card inserted, unable to get total space.\n");
        printf("Error: %s\n", fat32_error_string(result));
        return;
    }
    char buffer[32];
    fat32_get_volume_name(buffer, sizeof(buffer));
    printf("SD card inserted, ready to use.\n");
    printf("  Volume name: %s\n", buffer[0] ? buffer : "No volume label");
    get_str_size(buffer, sizeof(buffer), total_space);
    printf("  Capacity: %s\n", buffer);
    bool is_sdhc = sd_is_sdhc();
    printf("  Type: %s\n", is_sdhc ? "SDHC" : "SDSC");
    get_str_size(buffer, sizeof(buffer), fat32_get_cluster_size());
    printf("  Cluster size: %s\n", buffer);
}

void sd_free()
{
    uint64_t free_space;
    sd_error_t result = fat32_get_free_space(&free_space);

    if (result == SD_OK)
    {
        char size_buffer[32];
        get_str_size(size_buffer, sizeof(size_buffer), free_space);
        printf("Free space on SD card: %s\n", size_buffer);
    }
    else
    {
        printf("Error: %s\n", fat32_error_string(result));
    }
}

void cd(void)
{
    cd_dirname("/"); // Default to root directory
}

void cd_dirname(const char *dirname)
{
    if (dirname == NULL || strlen(dirname) == 0)
    {
        printf("Error: No directory specified.\n");
        printf("Usage: cd <dirname>\n");
        printf("Example: cd /mydir\n");
        return;
    }

    sd_error_t result = fat32_set_current_dir(dirname);
    if (result != SD_OK)
    {
        printf("Error: %s\n", fat32_error_string(result));
        return;
    }
}

void sd_pwd()
{
    char current_dir[FAT32_MAX_PATH_LEN];
    sd_error_t result = fat32_get_current_dir(current_dir, sizeof(current_dir));
    if (result != SD_OK)
    {
        printf("Error: %s\n", fat32_error_string(result));
        return;
    }
    printf("%s\n", current_dir);
}

void dir()
{
    sd_dir_dirname("."); // Show root directory
}

void sd_dir_dirname(const char *dirname)
{
    fat32_dir_t dir;
    fat32_entry_t dir_entry;

    fat32_error_t result = fat32_dir_open(&dir, dirname);
    if (result != SD_OK)
    {
        printf("Error: %s\n", fat32_error_string(result));
        return;
    }

    do
    {
        result = fat32_dir_read(&dir, &dir_entry);
        if (result != SD_OK)
        {
            printf("Error: %s\n", fat32_error_string(result));
            return;
        }
        if (dir_entry.filename[0])
        {
            if (dir_entry.attr & (FAT32_ATTR_VOLUME_ID | FAT32_ATTR_HIDDEN | FAT32_ATTR_SYSTEM))
            {
                // It's a volume label, hidden file, or system file, skip it
                continue;
            }
            else if (dir_entry.attr & FAT32_ATTR_DIRECTORY)
            {
                // It's a directory, append '/' to the name
                printf("%s/\n", dir_entry.filename);
            }
            else
            {
                char size_buffer[16];
                get_str_size(size_buffer, sizeof(size_buffer), dir_entry.size);
                printf("%-28s %10s\n", dir_entry.filename, size_buffer);
            }
        }
    } while (dir_entry.filename[0]);

    fat32_dir_close(&dir);
}

void sd_more()
{
    printf("Error: No filename specified.\n");
    printf("Usage: more <filename>\n");
    printf("Example: more readme.txt\n");
}

void sd_read_filename(const char *filename)
{
    if (filename == NULL || strlen(filename) == 0)
    {
        printf("Error: No filename specified.\n");
        printf("Usage: sd_read <filename>\n");
        printf("Example: sd_read readme.txt\n");
        return;
    }

    FILE *fp;
    fp = fopen(filename, "r");
    if (fp == NULL)
    {
        printf("Cannot open file '%s':\n%s\n", filename, strerror(errno));
        return;
    }

    // Read and display the file content with pagination
    char buffer[1024]; // Read in larger chunks
    size_t bytes_read;
    uint32_t total_bytes_read = 0;
    int line_count = 0;
    bool user_quit = false;

    printf("\033[2J\033[H");

    while (!user_quit && !feof(fp))
    {
        bytes_read = fread(buffer, 1, sizeof(buffer) - 1, fp);
        if (bytes_read == 0)
        {
            if (ferror(fp))
            {
                printf("Error reading file '%s':\n%s\n", filename, strerror(errno));
            }
            break;
        }
        if (bytes_read < sizeof(buffer) - 1)
        {
            buffer[bytes_read] = '\0'; // Null terminate the string
        }
        else
        {
            buffer[sizeof(buffer) - 1] = '\0'; // Ensure null termination
        }
        total_bytes_read += bytes_read;

        // Display the content line by line
        char *line_start = buffer;
        char *line_end;

        while ((line_end = strchr(line_start, '\n')) != NULL || line_start < buffer + bytes_read)
        {
            if (line_end == NULL)
            {
                // Last line without newline
                printf("%s", line_start);
                break;
            }

            // Print the line (including newline)
            *line_end = '\0';
            printf("%s\n", line_start);
            size_t line_len = strlen(line_start);
            int screen_lines = (int)((line_len + 40) / 40); // Round up
            if (screen_lines == 0)
            {
                screen_lines = 1;
            }
            line_count += screen_lines;
            // Check if we need to pause
            if (line_count > 30)
            {
                printf("More?");
                char ch = getchar();
                if (ch == 'q' || ch == 'Q')
                {
                    user_quit = true;
                    printf("\n");
                    break;
                }
                printf("\033[2J\033[H");
                line_count = 0;
            }

            line_start = line_end + 1;
        }

        if (user_quit)
            break;
    }

    fclose(fp);
}

void sd_mkfile()
{
    printf("Error: No filename specified.\n");
    printf("Usage: mkfile <filename>\n");
    printf("Example: mkfile newfile.txt\n");
}

void sd_mkfile_filename(const char *filename)
{
    if (filename == NULL || strlen(filename) == 0)
    {
        printf("Error: No filename specified.\n");
        printf("Usage: mkfile <filename>\n");
        printf("Example: mkfile newfile.txt\n");
        return;
    }

    FILE *fp;
    fp = fopen(filename, "wx+");
    if (fp == NULL)
    {
        printf("Cannot create file '%s':\n%s\n", filename, strerror(errno));
        return;
    }

    // Get lines of text and write them to the file
    // If a line is only a dot, stop reading, and close the file
    printf("Enter text to write to the file,\nfinish with a single dot:\n");
    char line[38];
    uint32_t total_bytes_written = 0;
    while (true)
    {
        printf("> ");
        readline(line, sizeof(line));
        if (strcmp(line, ".") == 0)
        {
            break; // Stop reading on a single dot
        }
        size_t bytes_written;
        size_t remaining_space = sizeof(line) - strlen(line) - 1;
        strncat(line, "\n", remaining_space);
        bytes_written = fwrite(line, 1, strlen(line), fp);
        if (bytes_written < strlen(line))
        {
            printf("Warning: Not all bytes written!\n");
        }
        total_bytes_written += bytes_written;
    }

    fclose(fp);

    printf("File '%s' created\nwith %u bytes written.\n", filename, total_bytes_written);
}

void sd_mkdir()
{
    printf("Error: No directory name specified.\n");
    printf("Usage: mkdir <dirname>\n");
    printf("Example: mkdir newdir\n");
}

void sd_mkdir_filename(const char *dirname)
{
    if (dirname == NULL || strlen(dirname) == 0)
    {
        printf("Error: No directory name specified.\n");
        printf("Usage: mkdir <dirname>\n");
        printf("Example: mkdir newdir\n");
        return;
    }

    fat32_dir_t dir;
    fat32_error_t result = fat32_dir_create(&dir, dirname);
    if (result != SD_OK)
    {
        printf("Error: %s\n", fat32_error_string(result));
        return;
    }

    printf("Directory '%s' created.\n", dirname);
    fat32_dir_close(&dir);
}

void sd_rm()
{
    printf("Error: No filename specified.\n");
    printf("Usage: rm <filename>\n");
    printf("Example: rm oldfile.txt\n");
}

void sd_rm_filename(const char *filename)
{
    if (filename == NULL || strlen(filename) == 0)
    {
        printf("Error: No filename specified.\n");
        printf("Usage: rm <filename>\n");
        printf("Example: rm oldfile.txt\n");
        return;
    }

    fat32_error_t result = fat32_file_delete(filename);
    if (result != FAT32_OK)
    {
        printf("Error: %s\n", fat32_error_string(result));
        return;
    }

    printf("File '%s' removed.\n", filename);
}

void sd_rmdir()
{
    printf("Error: No directory name specified.\n");
    printf("Usage: rmdir <dirname>\n");
    printf("Example: rmdir olddir\n");
}

void sd_rmdir_dirname(const char *dirname)
{
    if (dirname == NULL || strlen(dirname) == 0)
    {
        printf("Error: No directory name specified.\n");
        printf("Usage: rmdir <dirname>\n");
        printf("Example: rmdir olddir\n");
        return;
    }

    fat32_error_t result = fat32_dir_delete(dirname);
    if (result != FAT32_OK)
    {
        printf("Error: %s\n", fat32_error_string(result));
        return;
    }

    printf("Directory '%s' removed.\n", dirname);
}
