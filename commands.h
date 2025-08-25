#pragma once

// Command structure for function pointer table
typedef struct {
    const char* name;
    void (*function)(void);
    const char* description;
} command_t;

extern uint8_t columns; // Global variable for terminal width

// Forward declarations
void backlight(void);
void battery(void);
void beep(void);
void box(void);
void bye(void);
void cd(void);
void clearscreen(void);
void dir(void);
void play(void);
void run_command(const char *command);
void show_command_library(void);
void test(void);
void width(void);
void width_set(const char *width_str);
void reset();

// SD card commands
void sd_pwd(void);
void cd_dirname(const char *dirname);
void sd_dir_dirname(const char *dirname);
void sd_free(void);
void sd_more(void);
void sd_read_filename(const char *filename);
void sd_status(void);
void sd_mkfile(void);
void sd_mkfile_filename(const char *filename);
void sd_mkdir(void);
void sd_mkdir_filename(const char *dirname);
void sd_mv(void);
void sd_mv_filename(const char *oldname, const char *newname);
void sd_rm(void);
void sd_rm_filename(const char *filename);
void sd_rmdir(void);
void sd_rmdir_dirname(const char *dirname);

