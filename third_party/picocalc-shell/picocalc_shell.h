/* picocalc_shell.h — return from a program to the PicoCalc shell, and
 * the time it keeps.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later OR MIT
 *
 * This one header is offered under the MIT licence as well as the GPL,
 * so that a program under any licence can include it (design.md §9).
 * Copyright (c) 2026 Blair Leduc.
 *
 * For a program built with the Pico SDK for an RP2350 PicoCalc under UF2
 * Loader 2.3 or later, with the shell installed as /BOOT2350.UF2. The
 * shell started the program by flashing it and rebooting; this reboots
 * the other way, telling the loader's stage3 to load the shell from the
 * card instead of starting the program again.
 *
 *     #include "picocalc_shell.h"
 *     ...
 *     save_settings();              // finish every card write first
 *     if (!picocalc_shell_exit())
 *         show_message("Cannot return to the shell");
 *
 * The program must close its files first: the reboot does not wait for
 * them. Holding Up, F1 or F5 at power-on also returns to the shell, for
 * any program. With the shell's patched loader (design.md M7), power-on
 * gives the shell with no key held, and Up, F1 or F5 the program; a
 * program's own watchdog_reboot(0, 0, 0) starts it again under either
 * loader, and holding Up, F1 or F5 through that reboot gives the shell.
 *
 * A program can also be installed without the loader, straight from
 * BOOTSEL; the call returns false there too, and the program can do
 * something else instead, such as power off through its own southbridge
 * driver. picocalc_shell_loader_present() and
 * picocalc_shell_card_present() tell it beforehand, to name its menu
 * item.
 *
 * The time (design.md §3.8): the shell keeps UTC in the RP2350's
 * always-on timer, which survives the reboot into the program, and a
 * record of it in POWMAN's scratch registers 0-2:
 *
 *     int64_t utc_ms;
 *     int offset_min;
 *     if (picocalc_shell_time(&utc_ms, &offset_min))
 *         show_clock(utc_ms / 1000 + offset_min * 60);   // local time
 *
 * These two functions are here when the program links hardware_powman
 * (target_link_libraries(... hardware_powman)), so a program that only
 * returns to the shell needs nothing more. A program may read the time
 * at any moment; it must not stop the timer, change its tick source,
 * set it except through picocalc_shell_set_time(), or write POWMAN
 * scratch 0-2.
 *
 * The loader's values below are UF2 Loader's
 * (common/bootloader/proginfo.c); the shell's src/port/loader.c checks
 * it agrees with them, and src/port/clock.c with the clock's.
 */
#ifndef PICOCALC_SHELL_H
#define PICOCALC_SHELL_H

#include <stdbool.h>

#include "hardware/gpio.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/watchdog.h"
#include "hardware/watchdog.h"
#include "pico/time.h"

/* The card's detect switch: low with a card in, so it needs a pull-up. */
#define PICOCALC_SHELL_CARD_DETECT_PIN 22u

/* A command to stage3 in the watchdog's scratch registers: the magic in
 * scratch[0], the mode in [1], an argument in [2]. They survive the
 * watchdog's reboot, and stage3 clears the magic once it has read them. */
#define PICOCALC_SHELL_LOADER_MAGIC 0xe98cc638u
#define PICOCALC_SHELL_BOOT_SD      1u    /* load /BOOT2350.UF2 into SRAM */

/* Whether a card is in the slot. */
static inline bool picocalc_shell_card_present(void) {
    gpio_init(PICOCALC_SHELL_CARD_DETECT_PIN);
    gpio_set_dir(PICOCALC_SHELL_CARD_DETECT_PIN, GPIO_IN);
    gpio_pull_up(PICOCALC_SHELL_CARD_DETECT_PIN);
    busy_wait_us(100);   /* the pull-up's settling */
    return !gpio_get(PICOCALC_SHELL_CARD_DETECT_PIN);
}

/* Whether the program was started by UF2 Loader. stage3 starts it from
 * the app partition, after the loader in flash, so the boot ROM maps a
 * flash offset other than 0 at 0x10000000; a program copied on in
 * BOOTSEL runs from offset 0. (It shows a partition boot, not stage3 by
 * name; without stage3, the reboot below would only start the program
 * again.) */
static inline bool picocalc_shell_loader_present(void) {
    return (qmi_hw->atrans[0] & QMI_ATRANS0_BASE_BITS) != 0;
}

/* Return to the shell: stage3 loads /BOOT2350.UF2 instead of the app.
 * Returns false, and does nothing, if UF2 Loader did not start the
 * program, or there is no card to load the shell from: without one,
 * stage3 would drop to BOOTSEL and the PicoCalc would look dead.
 * Otherwise it does not return. */
static inline bool picocalc_shell_exit(void) {
    if (!picocalc_shell_loader_present() || !picocalc_shell_card_present()) return false;
    watchdog_hw->scratch[1] = PICOCALC_SHELL_BOOT_SD;
    watchdog_hw->scratch[2] = 0;
    watchdog_hw->scratch[0] = PICOCALC_SHELL_LOADER_MAGIC;
    watchdog_reboot(0, 0, 0);
    for (;;) tight_loop_contents();
}

#if defined(__has_include)
#if __has_include("hardware/powman.h")
#define PICOCALC_SHELL_HAS_TIME 1
#endif
#endif

#ifdef PICOCALC_SHELL_HAS_TIME
#include <stdint.h>

#include "hardware/powman.h"

/* The record: scratch[0] the magic, scratch[2] ~(scratch[0] ^
 * scratch[1]) as a check, and in scratch[1] these, among the shell's
 * own. */
#define PICOCALC_SHELL_CLOCK_MAGIC   0x434c4b31u   /* "CLK1" */
#define PICOCALC_SHELL_CLOCK_OFFSET  0x0000ffffu   /* minutes from UTC, signed */
#define PICOCALC_SHELL_CLOCK_SOURCE  0x00070000u   /* who set it */
#define PICOCALC_SHELL_CLOCK_PROGRAM 0x00040000u   /* a program */
#define PICOCALC_SHELL_CLOCK_SET     0x00080000u
#define PICOCALC_SHELL_CLOCK_RTC_DUE 0x00400000u   /* the shell writes the DS3231 */

/* scratch[1], or 0 if the record is not whole: power-on cleared it. */
static inline uint32_t picocalc_shell_clock_record(void) {
    uint32_t magic = powman_hw->scratch[0], bits = powman_hw->scratch[1];
    if (magic != PICOCALC_SHELL_CLOCK_MAGIC || powman_hw->scratch[2] != ~(magic ^ bits)) return 0;
    return bits;
}

/* The time the shell keeps: UTC milliseconds since 1970, and the local
 * offset in minutes when the shell started the program (its TZ then).
 * Returns false, and leaves both alone, if the clock is not set: as
 * after power-on with the stock loader, which starts the last program
 * without the shell. */
static inline bool picocalc_shell_time(int64_t *utc_ms, int *offset_min) {
    uint32_t bits = picocalc_shell_clock_record();
    if (!(bits & PICOCALC_SHELL_CLOCK_SET) || !powman_timer_is_running()) return false;
    *utc_ms = (int64_t)powman_timer_get_ms();
    *offset_min = (int16_t)(bits & PICOCALC_SHELL_CLOCK_OFFSET);
    return true;
}

/* Set the time, for a program that learns it itself (its own NTP or
 * RTC). The shell writes the DS3231, if there is one, when it comes
 * back. The offset is left as the shell gave it, 0 if it did not. */
static inline void picocalc_shell_set_time(int64_t utc_ms) {
    powman_timer_set_1khz_tick_source_xosc();
    powman_timer_set_ms((uint64_t)utc_ms);
    powman_timer_start();
    uint32_t bits = picocalc_shell_clock_record() & ~PICOCALC_SHELL_CLOCK_SOURCE;
    bits |= PICOCALC_SHELL_CLOCK_SET | PICOCALC_SHELL_CLOCK_PROGRAM | PICOCALC_SHELL_CLOCK_RTC_DUE;
    powman_hw->scratch[0] = PICOCALC_SHELL_CLOCK_MAGIC;
    powman_hw->scratch[1] = bits;
    powman_hw->scratch[2] = ~(PICOCALC_SHELL_CLOCK_MAGIC ^ bits);
}
#endif /* PICOCALC_SHELL_HAS_TIME */

#endif /* PICOCALC_SHELL_H */
