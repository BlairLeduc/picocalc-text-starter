/* fat32_probe.c - small tests for drivers/fat32.c, run on a PC.
 *
 * Author: Thomas Dzubin.
 *
 * Nothing from the Pico SDK is needed: the SD card is a block of RAM that this file
 * formats as FAT32, and stubs/ holds empty stand-ins for the three SDK headers that
 * fat32.c includes. Build and run from this folder (MinGW, Linux or macOS):
 *
 *   gcc -O1 -std=gnu99 -Istubs -I../../drivers fat32_probe.c ../../drivers/fat32.c -o fat32_probe
 *   ./fat32_probe
 *
 * The Pico toolchain has an unsigned char and a PC has a signed one: add -funsigned-char
 * to see what the device does.
 *
 * Each test starts from a freshly formatted card and prints PASS or FAIL with what was
 * seen. The exit status is the number of failed tests. Only the public API of fat32.h
 * is used.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "sdcard.h"
#include "fat32.h"

/* ------------------------------------------------------------------ */
/*  A RAM "SD card" with a FAT32 volume on it                           */
/* ------------------------------------------------------------------ */
#define CARD_SECTORS     600000u        /* 293 MB: FAT32 needs at least 65525 clusters */
#define SECTORS_PER_CLUS 8u            /* 4 KB clusters */
#define RESERVED         32u

static uint8_t *card;
static unsigned long sd_reads, sd_writes;

void sd_init(void) {}
sd_error_t sd_card_init(void) { return SD_OK; }
bool sd_card_present(void) { return true; }
bool sd_is_sdhc(void) { return true; }
const char *sd_error_string(sd_error_t e) { (void)e; return "sd error"; }

sd_error_t sd_read_block(uint32_t block, uint8_t *buffer)
{
    if (block >= CARD_SECTORS)
        return SD_ERROR_READ_FAILED;
    memcpy(buffer, card + (size_t)block * 512u, 512u);
    sd_reads++;
    return SD_OK;
}

sd_error_t sd_write_block(uint32_t block, const uint8_t *buffer)
{
    if (block >= CARD_SECTORS)
        return SD_ERROR_WRITE_FAILED;
    memcpy(card + (size_t)block * 512u, buffer, 512u);
    sd_writes++;
    return SD_OK;
}

sd_error_t sd_read_blocks(uint32_t start, uint32_t n, uint8_t *buffer)
{
    while (n--) {
        sd_error_t e = sd_read_block(start++, buffer);

        if (e != SD_OK)
            return e;
        buffer += 512;
    }
    return SD_OK;
}

sd_error_t sd_write_blocks(uint32_t start, uint32_t n, const uint8_t *buffer)
{
    while (n--) {
        sd_error_t e = sd_write_block(start++, buffer);

        if (e != SD_OK)
            return e;
        buffer += 512;
    }
    return SD_OK;
}

static void put16(uint8_t *p, unsigned v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, v & 0xFFFFu); put16(p + 2, v >> 16); }

/* Format the RAM card as a blank FAT32 volume (no partition table). */
static void format_card(void)
{
    uint32_t fat = CARD_SECTORS / SECTORS_PER_CLUS / 128u + 2u;
    uint8_t *bs, *fs, *f;

    free(card);
    card = calloc(CARD_SECTORS, 512u);
    bs = card;
    memcpy(bs, "\xEB\x58\x90MSDOS5.0", 11);
    put16(bs + 11, 512);
    bs[13] = SECTORS_PER_CLUS;
    put16(bs + 14, RESERVED);
    bs[16] = 2;                                 /* two FATs */
    bs[21] = 0xF8;
    put16(bs + 24, 63);
    put16(bs + 26, 255);
    put32(bs + 32, CARD_SECTORS);
    put32(bs + 36, fat);
    put32(bs + 44, 2);                          /* root directory cluster */
    put16(bs + 48, 1);                          /* FSInfo sector */
    put16(bs + 50, 6);                          /* backup boot sector */
    bs[66] = 0x29;
    memcpy(bs + 71, "NO NAME    FAT32   ", 19);
    bs[510] = 0x55;
    bs[511] = 0xAA;
    memcpy(card + 6 * 512u, bs, 512u);
    fs = card + 512u;
    put32(fs, 0x41615252u);
    put32(fs + 484, 0x61417272u);
    put32(fs + 488, 0xFFFFFFFFu);               /* free count unknown */
    put32(fs + 492, 0xFFFFFFFFu);               /* next free unknown */
    put32(fs + 508, 0xAA550000u);
    f = card + RESERVED * 512u;
    put32(f, 0x0FFFFFF8u);
    put32(f + 4, 0x0FFFFFFFu);
    put32(f + 8, 0x0FFFFFFFu);                  /* the root directory is one cluster */
    memcpy(f + fat * 512u, f, 12);              /* second FAT */
    sd_reads = sd_writes = 0;
    fat32_unmount();
    fat32_init();
}

/* ------------------------------------------------------------------ */
/*  Helpers                                                             */
/* ------------------------------------------------------------------ */
static int failures;

static void verdict(const char *test, int ok, const char *detail)
{
    printf("%-4s %-34s %s\n", ok ? "PASS" : "FAIL", test, detail);
    if (!ok)
        failures++;
}

static int make_file(const char *path, unsigned size, uint8_t fill)
{
    fat32_file_t f;
    uint8_t chunk[512];
    size_t put;

    memset(chunk, fill, sizeof chunk);
    if (fat32_create(&f, path) != FAT32_OK)
        return 1;
    while (size) {
        size_t n = size < sizeof chunk ? size : sizeof chunk;

        if (fat32_write(&f, chunk, n, &put) != FAT32_OK || put != n)
            return 1;
        size -= (unsigned)n;
    }
    fat32_close(&f);
    return 0;
}

/* Open a file and check its size and first byte. Returns 0 if all is as expected. */
static int check_file(const char *path, unsigned size, uint8_t fill)
{
    fat32_file_t f;
    uint8_t b = 0;
    size_t got = 0;

    if (fat32_open(&f, path) != FAT32_OK)
        return 1;
    if (fat32_size(&f) != size) {
        fat32_close(&f);
        return 2;
    }
    if (size && (fat32_read(&f, &b, 1, &got) != FAT32_OK || got != 1 || b != fill)) {
        fat32_close(&f);
        return 3;
    }
    fat32_close(&f);
    return 0;
}

static int exists(const char *path)
{
    fat32_file_t f;

    if (fat32_open(&f, path) != FAT32_OK)
        return 0;
    fat32_close(&f);
    return 1;
}

/* Count the entries a directory listing returns and note any with an unprintable name. */
static unsigned list_dir(const char *path, int *garbage, char *names, size_t names_size)
{
    fat32_file_t d;
    fat32_entry_t e;
    unsigned count = 0;
    size_t used = 0;

    *garbage = 0;
    names[0] = 0;
    if (fat32_open(&d, path) != FAT32_OK)
        return 0;
    while (fat32_dir_read(&d, &e) == FAT32_OK && e.filename[0]) {
        const unsigned char *p;

        for (p = (const unsigned char *)e.filename; *p; p++)
            if (*p < 32 || *p > 126)
                *garbage = 1;
        if (strcmp(e.filename, ".") && strcmp(e.filename, "..")) {
            count++;
            if (used + strlen(e.filename) + 2 < names_size)
                used += (size_t)sprintf(names + used, "%s ", e.filename);
        }
    }
    fat32_close(&d);
    return count;
}

/* Print the raw directory entries of the root directory (first cluster), for diagnosis. */
static int verbose;

static void dump_root(const char *title)
{
    uint32_t fat = CARD_SECTORS / SECTORS_PER_CLUS / 128u + 2u;
    const uint8_t *d = card + (size_t)(RESERVED + 2u * fat) * 512u;
    unsigned i, k;

    if (!verbose)
        return;
    printf("    raw root directory, %s:%s", title, "\n");
    for (i = 0; i < 16; i++, d += 32) {
        if (d[0] == 0)
            break;
        printf("    %2u: ", i);
        if (d[11] == 0x0F) {
            printf("LFN seq %02X chk %02X  \"", d[0], d[13]);
            for (k = 0; k < 10; k += 2)
                printf("%c", d[1 + k] >= 32 && d[1 + k] < 127 ? d[1 + k] : '.');
            printf("...\"");
        } else {
            printf("8.3 first byte %02X  \"%.11s\" attr %02X", d[0], (const char *)d, d[11]);
        }
        printf("\n");
    }
}

/* ------------------------------------------------------------------ */
/*  The tests                                                           */
/* ------------------------------------------------------------------ */

/* Delete long-named files and create them again under the same names, several times,
 * the way a program that rewrites its data files does. */
static void test_recreate(void)
{
    static const char *const name[3] = { "Long-File-Name-Number-One.bin",
                                         "Long-File-Name-Number-Two.bin",
                                         "Long-File-Name-Number-Three.bin" };
    char detail[700], listing[400];
    int cycle, i, bad = 0, garbage;
    unsigned count;

    format_card();
    for (cycle = 0; cycle < 6 && !bad; cycle++) {
        for (i = 0; i < 3 && !bad; i++) {
            fat32_delete(name[i]);                          /* fine if it is not there */
            if (make_file(name[i], 2000u + (unsigned)i, (uint8_t)(0x40 + cycle)))
                bad = 1;
        }
        for (i = 0; i < 3 && !bad; i++)
            if (check_file(name[i], 2000u + (unsigned)i, (uint8_t)(0x40 + cycle)))
                bad = 1;
        if (bad)
            sprintf(detail, "cycle %d: file %d cannot be opened or has the wrong content",
                    cycle + 1, i);
    }
    count = list_dir("/", &garbage, listing, sizeof listing);
    if (!bad && count != 3) {
        bad = 1;
        sprintf(detail, "listing shows %u entries, expected 3: %s", count, listing);
    }
    if (!bad && garbage) {
        bad = 1;
        sprintf(detail, "listing has unprintable names: %s", listing);
    }
    if (!bad)
        strcpy(detail, "6 delete + create cycles of 3 long names, all opened and listed");
    verdict("delete then create the same name", !bad, detail);
}

/* Rename a long-named file to another long name. */
static void test_rename(void)
{
    char detail[700], listing[400];
    fat32_error_t e;
    int garbage, bad = 0;
    unsigned count;

    format_card();
    if (make_file("Original-Long-Name-File.dat", 3000, 0x31) ||
        make_file("Second-Long-Name-File.dat", 1500, 0x32)) {
        verdict("rename a long-named file", 0, "setup failed");
        return;
    }
    e = fat32_rename("Original-Long-Name-File.dat", "Renamed-Long-Name-File.dat");
    if (e != FAT32_OK) {
        sprintf(detail, "first rename returned: %s", fat32_error_string(e));
        bad = 1;
    } else if (check_file("Renamed-Long-Name-File.dat", 3000, 0x31)) {
        strcpy(detail, "the renamed file cannot be opened or has the wrong content");
        bad = 1;
    } else if (exists("Original-Long-Name-File.dat")) {
        strcpy(detail, "the old name still exists");
        bad = 1;
    }
    if (!bad) {
        e = fat32_rename("Second-Long-Name-File.dat", "Another-Renamed-Long-Name.dat");
        if (e != FAT32_OK) {
            sprintf(detail, "second rename returned: %s", fat32_error_string(e));
            bad = 1;
        } else if (check_file("Another-Renamed-Long-Name.dat", 1500, 0x32)) {
            strcpy(detail, "the second renamed file cannot be opened or has wrong content");
            bad = 1;
        }
    }
    if (!bad) {
        count = list_dir("/", &garbage, listing, sizeof listing);
        if (count != 2 || garbage) {
            sprintf(detail, "listing: %u entries (expected 2)%s: %s", count,
                    garbage ? ", unprintable names" : "", listing);
            bad = 1;
        }
    }
    if (!bad)
        strcpy(detail, "two renames, both files open with the right content");
    verdict("rename a long-named file", !bad, detail);
}

/* After deleting a long-named file the listing must not show it, or anything odd. */
static void test_listing_after_delete(void)
{
    char detail[700], listing[400];
    int garbage;
    unsigned count;

    format_card();
    make_file("This-Is-A-Long-File-Name-One.txt", 100, 0x41);
    make_file("This-Is-A-Long-File-Name-Two.txt", 100, 0x42);
    make_file("short.txt", 100, 0x43);
    dump_root("before the delete");
    fat32_delete("This-Is-A-Long-File-Name-One.txt");
    dump_root("after the delete");
    count = list_dir("/", &garbage, listing, sizeof listing);
    if (count == 2 && !garbage && exists("This-Is-A-Long-File-Name-Two.txt") &&
        exists("short.txt") && !exists("This-Is-A-Long-File-Name-One.txt"))
        strcpy(detail, "two files listed, the deleted one is gone");
    else
        sprintf(detail, "%u entries%s: %s", count, garbage ? " (unprintable name)" : "", listing);
    verdict("listing after deleting a long name", count == 2 && !garbage, detail);
}

/* Writing a file piece by piece: the number of sector reads should stay about the size
 * of the data, but each fat32_write() call walks the file's cluster chain from its start. */
static void test_write_cost(void)
{
    static uint8_t block[4096];
    fat32_file_t f;
    size_t put;
    unsigned i, pieces = 256;                   /* 1 MB */
    unsigned long first_reads, last_reads, r;
    char detail[200];

    format_card();
    memset(block, 0x5A, sizeof block);
    if (fat32_create(&f, "bigfile.bin") != FAT32_OK) {
        verdict("cost of writing a growing file", 0, "cannot create");
        return;
    }
    first_reads = last_reads = 0;
    for (i = 0; i < pieces; i++) {
        r = sd_reads;
        if (fat32_write(&f, block, sizeof block, &put) != FAT32_OK || put != sizeof block)
            break;
        if (i == 15)
            first_reads = sd_reads - r;
        if (i == pieces - 1)
            last_reads = sd_reads - r;
    }
    fat32_close(&f);
    sprintf(detail, "one 4 KB write costs %lu sector reads at 64 KB into the file, %lu at 1 MB",
            first_reads, last_reads);
    /* A 4 KB write needs about 8 data sector reads; a cost that grows with the file
     * means the cluster chain is walked from the start on every call. */
    verdict("cost of writing a growing file", last_reads < first_reads * 3 + 32, detail);
}

/* File dates: none without a time source; with one, a file is dated when made and
 * when written, a renamed file keeps all its dates (or none), a new folder is dated,
 * and a source that doesn't know the time, or gives one FAT can't hold, leaves dates
 * alone. */
static int clock_year = 2026, clock_day = 24, clock_known = 1;

static bool test_clock(uint16_t *date, uint16_t *time)
{
    if (!clock_known)
        return false;
    return fat32_pack_time(clock_year, 10, clock_day, 15, 30, 42, date, time);
}

/* The date and time fat32_dir_read() gives for name in "/", and the creation date
 * from the entry on the card; all 0xFFFF if it isn't there. */
static void entry_date(const char *name, unsigned *date, unsigned *time, unsigned *created)
{
    fat32_file_t d;
    fat32_entry_t e;

    *date = *time = *created = 0xFFFFu;
    if (fat32_open(&d, "/") != FAT32_OK)
        return;
    while (fat32_dir_read(&d, &e) == FAT32_OK && e.filename[0])
        if (!strcmp(e.filename, name)) {
            const uint8_t *raw = card + (size_t)e.sector * 512u + e.offset;
            *date = e.date;
            *time = e.time;
            *created = raw[16] | raw[17] << 8;
        }
    fat32_close(&d);
}

static void test_dates(void)
{
    char detail[300];
    unsigned date, time, created, d24, d25, t;
    uint16_t pd, pt;
    int bad = 0;
    fat32_file_t f, dir;
    size_t put;

    fat32_pack_time(2026, 10, 24, 15, 30, 42, &pd, &pt);
    d24 = pd;
    t = pt;
    fat32_pack_time(2026, 10, 25, 15, 30, 42, &pd, &pt);
    d25 = pd;
    if (fat32_pack_time(1979, 12, 31, 23, 59, 58, &pd, &pt) ||
        fat32_pack_time(2108, 1, 1, 0, 0, 0, &pd, &pt) ||
        fat32_pack_time(2026, 13, 1, 0, 0, 0, &pd, &pt) ||
        fat32_pack_time(2026, 1, 1, 24, 0, 0, &pd, &pt)) {
        bad = 1;
        strcpy(detail, "fat32_pack_time() takes a time FAT can't hold");
    }
    format_card();

    fat32_set_time_source(NULL);
    clock_year = 2026;
    clock_day = 24;
    clock_known = 1;
    make_file("old.txt", 100u, 1);
    entry_date("old.txt", &date, &time, &created);
    if (!bad && (date != 0 || time != 0)) {
        bad = 1;
        sprintf(detail, "with no time source, a new file is dated %04x %04x, not 0", date, time);
    }

    fat32_set_time_source(test_clock);
    if (!bad) {
        if (fat32_rename("old.txt", "Old-Renamed.txt") != FAT32_OK)
            bad = 1, strcpy(detail, "can't rename the undated file");
        entry_date("Old-Renamed.txt", &date, &time, &created);
        if (!bad && (date != 0 || created != 0)) {
            bad = 1;
            sprintf(detail, "renamed, an undated file is dated %04x (made %04x), not 0", date, created);
        }
    }
    if (!bad) {
        make_file("Dated-Long-Name.txt", 100u, 2);
        entry_date("Dated-Long-Name.txt", &date, &time, &created);
        if (date != d24 || time != t) {
            bad = 1;
            sprintf(detail, "a new file is dated %04x %04x, expected %04x %04x", date, time, d24, t);
        }
    }
    if (!bad) {
        clock_day = 25;
        if (fat32_open(&f, "Dated-Long-Name.txt") != FAT32_OK || fat32_seek(&f, 100u) != FAT32_OK ||
            fat32_write(&f, "more", 4, &put) != FAT32_OK)
            bad = 1, strcpy(detail, "can't add to the file");
        fat32_close(&f);
        entry_date("Dated-Long-Name.txt", &date, &time, &created);
        if (!bad && date != d25) {
            bad = 1;
            sprintf(detail, "after a write next day the file is dated %04x, expected %04x", date, d25);
        }
    }
    if (!bad) {
        clock_day = 26;
        if (fat32_rename("Dated-Long-Name.txt", "Renamed.txt") != FAT32_OK)
            bad = 1, strcpy(detail, "can't rename it");
        entry_date("Renamed.txt", &date, &time, &created);
        if (!bad && date != d25) {
            bad = 1;
            sprintf(detail, "renamed, the file is dated %04x, expected to keep %04x", date, d25);
        }
        if (!bad && created != d24) {
            bad = 1;
            sprintf(detail, "renamed, the file was made %04x, expected to keep %04x", created, d24);
        }
    }
    if (!bad) {
        clock_day = 24;
        if (fat32_dir_create(&dir, "/Folder") != FAT32_OK)
            bad = 1, strcpy(detail, "can't make a folder");
        fat32_close(&dir);
        entry_date("Folder", &date, &time, &created);
        if (!bad && date != d24) {
            bad = 1;
            sprintf(detail, "a new folder is dated %04x, expected %04x", date, d24);
        }
    }
    if (!bad) {
        clock_known = 0;
        if (fat32_open(&f, "Renamed.txt") != FAT32_OK || fat32_write(&f, "x", 1, &put) != FAT32_OK)
            bad = 1, strcpy(detail, "can't write to the renamed file");
        fat32_close(&f);
        entry_date("Renamed.txt", &date, &time, &created);
        if (!bad && date != d25) {
            bad = 1;
            sprintf(detail, "with the time unknown, a write changed the date to %04x", date);
        }
    }
    if (!bad) {
        clock_known = 1;
        clock_year = 1979;
        if (fat32_open(&f, "Renamed.txt") != FAT32_OK || fat32_write(&f, "y", 1, &put) != FAT32_OK)
            bad = 1, strcpy(detail, "can't write to the renamed file");
        fat32_close(&f);
        entry_date("Renamed.txt", &date, &time, &created);
        if (!bad && date != d25) {
            bad = 1;
            sprintf(detail, "with the clock in 1979, a write changed the date to %04x", date);
        }
    }
    fat32_set_time_source(NULL);
    if (!bad)
        strcpy(detail, "none without a clock; made, written, renamed, folder, clock unknown or out of range");
    verdict("file dates", !bad, detail);
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    verbose = argc > 1;
    printf("fat32.c tests on a RAM card (%u sectors, %u sector(s) per cluster)\n\n",
           CARD_SECTORS, SECTORS_PER_CLUS);
    test_recreate();
    test_rename();
    test_listing_after_delete();
    test_dates();
    test_write_cost();
    printf("\n%d test(s) failed\n", failures);
    return failures;
}
