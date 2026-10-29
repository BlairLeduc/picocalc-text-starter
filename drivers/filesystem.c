//
//  PicoCalc file system, using ChaN's FatFs
//
//  FatFs does the work; this file mounts it, gives it the time, and turns
//  its result codes into messages. The disk layer is in diskio.c.
//

#include <time.h>

#include "pico/stdlib.h"

#if PICO_RP2350
#include "picocalc_shell.h" // Only the RP2350 runs the shell
#endif

#include "sdcard.h"
#include "filesystem.h"

// In diskio.c
extern void disk_detect_init(void);

static FATFS fs;
static bool fs_initialised = false;

#ifdef PICOCALC_SHELL_HAS_TIME
// The time the shell keeps (RP2350 only), in local time as the shell's
// time zone stood when it started this program
static bool shell_clock(uint16_t *date, uint16_t *time)
{
    int64_t utc_ms;
    int offset_min;
    if (!picocalc_shell_time(&utc_ms, &offset_min))
    {
        return false; // Not started from the shell, or its clock isn't set
    }
    time_t t = (time_t)(utc_ms / 1000) + offset_min * 60;
    struct tm tm;
    gmtime_r(&t, &tm);
    return fs_pack_time(tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                        tm.tm_hour, tm.tm_min, tm.tm_sec, date, time);
}

// The program's clock, for file dates (NULL: none)
static fs_time_source_t time_source = shell_clock;
#else
// The program's clock, for file dates (NULL: none)
static fs_time_source_t time_source = NULL;
#endif

void fs_set_time_source(fs_time_source_t source)
{
    time_source = source;
}

bool fs_pack_time(int year, int month, int day, int hour, int minute, int second,
                  uint16_t *date, uint16_t *time)
{
    if (year < 1980 || year > 2107 || month < 1 || month > 12 || day < 1 || day > 31 ||
        hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 59)
    {
        return false;
    }
    *date = (uint16_t)((year - 1980) << 9 | month << 5 | day);
    *time = (uint16_t)(hour << 11 | minute << 5 | second / 2);
    return true;
}

// FatFs asks for the time here when it dates a file: the date in the high
// 16 bits and the time in the low, or 0 when the program hasn't said.
DWORD get_fattime(void)
{
    uint16_t date, time;
    if (time_source && time_source(&date, &time))
    {
        return (DWORD)date << 16 | time;
    }
    return 0;
}

const char *fs_error_string(FRESULT result)
{
    switch (result)
    {
    case FR_OK:
        return "Success";
    case FR_DISK_ERR:
        return "SD card read or write failed";
    case FR_INT_ERR:
        return "File system internal error";
    case FR_NOT_READY:
        return "No SD card present";
    case FR_NO_FILE:
        return "File not found";
    case FR_NO_PATH:
        return "Directory not found";
    case FR_INVALID_NAME:
        return "Invalid path";
    case FR_DENIED:
        return "Access denied, directory not empty, or disk full";
    case FR_EXIST:
        return "File already exists";
    case FR_INVALID_OBJECT:
        return "File is not open (was the card changed?)";
    case FR_WRITE_PROTECTED:
        return "SD card is write protected";
    case FR_INVALID_DRIVE:
        return "Invalid drive";
    case FR_NOT_ENABLED:
        return "File system not mounted";
    case FR_NO_FILESYSTEM:
        return "Invalid SD card format";
    case FR_TIMEOUT:
        return "Timed out";
    case FR_LOCKED:
        return "File is already open";
    case FR_NOT_ENOUGH_CORE:
        return "Not enough memory";
    case FR_TOO_MANY_OPEN_FILES:
        return "Too many open files";
    case FR_INVALID_PARAMETER:
        return "Invalid parameter";
    default:
        return "Unknown error";
    }
}

void fs_init(void)
{
    if (fs_initialised)
    {
        return; // Already initialized
    }

    // Initialize the SD card
    sd_init();

    // Register the file system; FatFs mounts it on first use
    f_mount(&fs, "", 0);

    // Check if a SD card is present
    disk_detect_init();

    fs_initialised = true;
}
