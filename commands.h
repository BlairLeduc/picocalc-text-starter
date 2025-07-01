#pragma once

// Command structure for function pointer table
typedef struct {
    const char* name;
    void (*function)(void);
    const char* description;
} command_t;

// Forward declarations
void run_command(const char *command);
void help(void);
void backlight(void);
void battery(void);
void beep(void);
void box(void);
void bye(void);
void clearscreen(void);
void play(void);
void test(void);

void play_stereo_harmony_demo(void);
void play_stereo_melody_demo(void);

// SD card commands
void sd_status(void);
void sd_mount_cmd(void);  // Renamed to avoid conflict with sd_mount() from sdcard.h
void sd_list(void);
void sd_read_file(void);
void sd_read_filename(const char *filename);
void sd_debug_cmd(void);
void sd_dump_sector(void);

