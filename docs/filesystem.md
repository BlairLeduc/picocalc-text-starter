# File System

The file system on the SD card is [FatFs](http://elm-chan.org/fsw/ff/) by ChaN. It reads and writes FAT12, FAT16, FAT32 and exFAT, so most cards work without reformatting, including cards over 32 GB.

FatFs is not copied into this project. The Pico SDK includes it (in `lib/tinyusb/lib/fatfs/source`), and `CMakeLists.txt` builds that copy with this project's configuration, [drivers/fatfs/ffconf.h](../drivers/fatfs/ffconf.h). Its notes at the top list what is changed from the SDK's configuration.

There are three files of glue between FatFs and the PicoCalc:

- `drivers/diskio.c` – the disk layer FatFs calls to read and write sectors, using the [SD Card](sdcard.md) driver
- `drivers/filesystem.c` – sets up the card, gives FatFs the time, and turns FatFs result codes into messages
- `drivers/clib.c` – the C library's file functions (`fopen`, `fread`, `rename`, ...), on top of FatFs

There are two ways to use files:

- Include `drivers/clib.c` and use the C standard library: `fopen`, `fread`, `fwrite`, `fseek`, `fclose`, `remove`, `rename`, `stat` and `mkdir`.
- Include `drivers/filesystem.h` and call FatFs directly: `f_open`, `f_read`, `f_opendir`, `f_chdir` and so on. The [FatFs documentation](http://elm-chan.org/fsw/ff/) describes each function.

`picocalc_init()` calls `fs_init()`, so there is nothing to set up. The card is mounted the first time a file is used, and again after a card is changed. You don't need to call `f_mount()`. Paths need no drive number (`"/docs/readme.txt"`, not `"0:/docs/readme.txt"`), and may be relative to the current directory set with `f_chdir()`.

Long file names are UTF-8 C strings.

Some things to know:

- **Changing the card.** Files that were open when a card is taken out stay unusable: FatFs returns `FR_INVALID_OBJECT` (`EBADF` through the C library), so close them and open them again. The current directory goes back to the root.
- **Open files.** Up to 16 files can be open at once. A file can be open more than once to read, but not while it is open to write.
- **Free space.** `f_getfree()` reads the free count the card keeps. If the card has none, as after some computers write to it, FatFs counts the free clusters, which can take a few seconds on a large card.
- **Size.** FatFs adds about 27 KB of flash and 2 KB of RAM. Each open file is another 80 bytes, as open files share one sector buffer (`FF_FS_TINY`).


## Using it in your own project

Copy these files, and the FatFs part of `CMakeLists.txt` (the block after `pico_sdk_init()`, and `fatfs` in `target_link_libraries`):

- `drivers/fatfs/ffconf.h`
- `drivers/diskio.c`
- `drivers/filesystem.c`, `drivers/filesystem.h`
- `drivers/sdcard.c`, `drivers/sdcard.h`
- `drivers/clib.c`, if you want the C library's file functions
- `third_party/picocalc-shell/picocalc_shell.h`, and the `hardware_powman` part of `CMakeLists.txt`, if you want files dated with the shell's clock

Call `fs_init()` at start-up if you don't use `picocalc_init()`.


## fs_init

`void fs_init(void)`

Sets up the SD card and file system. `picocalc_init()` calls this for you.


## fs_error_string

`const char *fs_error_string(FRESULT result)`

Returns a short message for a FatFs result code, for showing to the user.


## fs_set_time_source

`void fs_set_time_source(fs_time_source_t source)`

FatFs dates a file when it is created and each time it is written, and dates a new folder when it is made. The PicoCalc has no clock of its own, so the time comes from elsewhere:

- On an RP2350 (Pico 2, Pico 2 W, Pico Plus 2), files are dated with the time the [PicoCalc shell](https://github.com/BlairLeduc/picocalc-shell) (v1.1 or later) keeps, in the shell's local time. There is nothing to set up. It works when the shell started the program and its clock is set. The shell's header is in `third_party/picocalc-shell`, and `CMakeLists.txt` links `hardware_powman` for it.
- On an RP2040, or when the shell's clock isn't set, dates are left as 0.

A program with its own clock, such as an RTC or NTP, can supply the time instead. `source` is a function, `bool source(uint16_t *date, uint16_t *time)`, that fills in a FAT date and time (local time; `fs_pack_time()` makes them) and returns true, or returns false when it doesn't know the time.

FatFs calls the function while a file is being written, so it must not call the `f_*` functions itself.

### Parameters

- source – the function that gives the time, or NULL to stop dating files


## fs_pack_time

`bool fs_pack_time(int year, int month, int day, int hour, int minute, int second, uint16_t *date, uint16_t *time)`

Packs a date and time into FAT's format. FAT keeps time to two seconds. Returns false, and leaves `date` and `time` alone, if any value is out of range.

### Parameters

- year – 1980 to 2107
- month – 1 to 12
- day – 1 to 31
- hour – 0 to 23
- minute – 0 to 59
- second – 0 to 59
- date – where to store the FAT date
- time – where to store the FAT time


# Moving from the FAT32 driver

Up to v0.16 this starter had its own FAT32 driver, `drivers/fat32.c`. If your project calls the `fat32_*` functions, you can stay on [v0.16](https://github.com/BlairLeduc/picocalc-text-starter/tree/v0.16), or move to FatFs:

| FAT32 driver | FatFs |
|---|---|
| `fat32_init()` | `fs_init()` (`picocalc_init()` calls it) |
| `fat32_mount()`, `fat32_is_ready()` | Not needed: FatFs mounts on first use |
| `fat32_file_t` | `FIL` for a file, `DIR` for a directory |
| `fat32_error_t`, `FAT32_OK` | `FRESULT`, `FR_OK` |
| `fat32_open(&f, path)` | `f_open(&f, path, FA_READ)` (add `FA_WRITE` to write) |
| `fat32_create(&f, path)` | `f_open(&f, path, FA_CREATE_NEW \| FA_WRITE)` |
| `fat32_read`, `fat32_write` | `f_read`, `f_write` (counts are `UINT`) |
| `fat32_seek`, `fat32_tell`, `fat32_size`, `fat32_eof` | `f_lseek`, `f_tell`, `f_size`, `f_eof` |
| `fat32_close(&f)` | `f_close(&f)`, or `f_closedir(&d)` for a directory |
| `fat32_delete(path)` | `f_unlink(path)` (files and empty directories) |
| `fat32_rename(old, new)` | `f_rename(old, new)` |
| `fat32_dir_create(&d, path)` | `f_mkdir(path)` |
| `fat32_open(&d, path)` + `fat32_dir_read(&d, &e)` | `f_opendir(&d, path)` + `f_readdir(&d, &info)`; the end is an empty `info.fname` |
| `fat32_set_current_dir`, `fat32_get_current_dir` | `f_chdir`, `f_getcwd` |
| `fat32_get_free_space`, `fat32_get_total_space` | `f_getfree` |
| `fat32_get_volume_name` | `f_getlabel` |
| `fat32_error_string` | `fs_error_string` |
| `fat32_set_time_source`, `fat32_pack_time` | `fs_set_time_source`, `fs_pack_time` (the same, renamed). On an RP2350 the shell's clock is now used without calling it |
