#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "ff.h"

// The file system on the SD card is ChaN's FatFs (http://elm-chan.org/fsw/ff/).
// Include this header and call the f_* functions directly, or include
// drivers/clib.c in your project and use fopen() and friends.
//
// fs_init() mounts the card the first time it is used, and again after it
// is swapped, so there is no need to call f_mount() yourself. Paths may be
// relative to the current directory (f_chdir) and need no drive number.

#define FS_MAX_PATH_LEN (260)

// Sets up the SD card and file system. picocalc_init() calls this.
void fs_init(void);

// A short message for a FatFs result code.
const char *fs_error_string(FRESULT result);

// File dates
//
// The PicoCalc has no clock of its own. On an RP2350, files are dated with
// the time the PicoCalc shell (v1.1 or later) keeps, when the shell started
// this program and its clock is set; there is nothing to do. Otherwise
// (an RP2040, or a program not started from the shell) dates are left as 0.
//
// A program with its own clock (an RTC, NTP) can supply the time instead:
// a function that fills in a FAT date and time (local time; fs_pack_time()
// makes them) and returns true, or returns false when it doesn't know the
// time. FatFs stamps a file when it is created and each time it is
// written, and a new folder when it is made. FatFs calls the function
// while a file is being written, so it must not call the f_* functions
// itself. fs_set_time_source(NULL) stops files being dated.
typedef bool (*fs_time_source_t)(uint16_t *date, uint16_t *time);
void fs_set_time_source(fs_time_source_t source);

// A FAT date and time: year 1980-2107, month 1-12, day 1-31, hour 0-23,
// minute 0-59, second 0-59 (FAT keeps it to two seconds). Returns false,
// and leaves both alone, if any is out of range.
bool fs_pack_time(int year, int month, int day, int hour, int minute, int second,
                  uint16_t *date, uint16_t *time);
