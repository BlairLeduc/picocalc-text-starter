/* sd_busy_probe.c - what drivers/sdcard.c does when a block takes a while to program.
 *
 * Author: Thomas Dzubin.
 *
 * An SD card that is programming a block holds its data line (MISO) low until it is
 * done, and ignores commands meanwhile. This file is a small model of a card in SPI
 * mode, with a simulated clock, that is busy for a chosen time after each write. The
 * driver's sdcard.c runs unchanged on top of it (with the Pico SDK calls replaced by
 * the functions below), and the program writes and reads back blocks and reports what
 * the driver returned.
 *
 * Only the mechanism is modelled: how long a real card is busy, and how long one SPI
 * byte takes (SPI_CALL_US below, the cost of one spi_write_read_blocking() call besides
 * the bits on the wire) are assumptions, not measurements.
 *
 * Build and run from this folder (MinGW, Linux or macOS):
 *
 *   gcc -O1 -std=gnu99 -Istubs -I../../drivers sd_busy_probe.c ../../drivers/sdcard.c -o sd_busy_probe
 *   ./sd_busy_probe
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "sdcard.h"

#define CARD_BLOCKS  512u
#define SPI_CALL_US  2u                 /* assumed cost of one SDK call, beyond the bits */

/* ------------------------------------------------------------------ */
/*  The fake card                                                       */
/* ------------------------------------------------------------------ */
static uint8_t  store[CARD_BLOCKS][512];
static unsigned baud;
static uint64_t sim_us;                 /* simulated time */
static uint64_t busy_until;
static uint32_t busy_ms;                /* how long the card is busy after each write */
static bool     cs_high = true;
static bool     card_ready;             /* ACMD41 done */
static unsigned acmd41_count;
static bool     app_cmd;

enum { S_CMD, S_ARGS, S_TOKEN, S_DATA, S_CRC } state;
static uint8_t  frame[6];
static unsigned frame_len, data_len, crc_len;
static uint8_t  wbuf[512];
static uint32_t wblock;

static uint8_t outq[600];
static unsigned out_head, out_tail;
static bool     busy_after_response;    /* start the busy time when the 0x05 is read */

static void push(uint8_t b) { outq[out_tail++ % sizeof outq] = b; }

static void command(void)
{
    unsigned cmd = frame[0] & 0x3Fu;
    uint32_t arg = ((uint32_t)frame[1] << 24) | ((uint32_t)frame[2] << 16) |
                   ((uint32_t)frame[3] << 8) | frame[4];

    push(0xFF);
    switch (cmd) {
    case 0:  push(0x01); break;
    case 8:  push(0x01); push(0); push(0); push(0x01); push(0xAA); break;
    case 55: push(card_ready ? 0x00 : 0x01); app_cmd = true; return;
    case 41:
        if (++acmd41_count >= 3)
            card_ready = true;
        push(card_ready ? 0x00 : 0x01);
        break;
    case 58: push(0x00); push(0xC0); push(0); push(0); push(0); break;
    case 17: {
        unsigned i;

        push(0x00);
        push(0xFF);
        push(0xFE);
        for (i = 0; i < 512; i++)
            push(arg < CARD_BLOCKS ? store[arg][i] : 0);
        push(0xFF);
        push(0xFF);
        break;
    }
    case 24:
        push(0x00);
        wblock = arg;
        state = S_TOKEN;
        app_cmd = false;
        return;
    default: push(0x04); break;         /* illegal command */
    }
    app_cmd = false;
}

/* One byte clocked through the card. */
static uint8_t card_byte(uint8_t in)
{
    uint8_t out = 0xFF;

    if (cs_high)
        return 0xFF;                    /* the card is not selected */
    if (sim_us < busy_until)
        return 0x00;                    /* busy: MISO held low, the input is ignored */
    if (out_head != out_tail) {
        out = outq[out_head++ % sizeof outq];
        if (busy_after_response && out == 0x05) {
            busy_until = sim_us + (uint64_t)busy_ms * 1000u;
            busy_after_response = false;
        }
    }
    switch (state) {
    case S_CMD:
        if ((in & 0xC0) == 0x40) {
            frame[0] = in;
            frame_len = 1;
            state = S_ARGS;
        }
        break;
    case S_ARGS:
        frame[frame_len++] = in;
        if (frame_len == 6) {
            state = S_CMD;
            command();
        }
        break;
    case S_TOKEN:
        if (in == 0xFE) {
            data_len = 0;
            state = S_DATA;
        }
        break;
    case S_DATA:
        wbuf[data_len++] = in;
        if (data_len == 512) {
            crc_len = 0;
            state = S_CRC;
        }
        break;
    case S_CRC:
        if (++crc_len == 2) {
            if (wblock < CARD_BLOCKS)
                memcpy(store[wblock], wbuf, 512);
            push(0x05);                 /* data accepted */
            busy_after_response = true;
            state = S_CMD;
        }
        break;
    }
    return out;
}

/* ------------------------------------------------------------------ */
/*  The Pico SDK calls that sdcard.c uses                               */
/* ------------------------------------------------------------------ */
void gpio_init(unsigned pin) { (void)pin; }
void gpio_set_dir(unsigned pin, bool out) { (void)pin; (void)out; }
void gpio_pull_up(unsigned pin) { (void)pin; }
void gpio_set_function(unsigned pin, unsigned function) { (void)pin; (void)function; }
bool gpio_get(unsigned pin) { (void)pin; return false; }       /* card present */
void gpio_put(unsigned pin, bool value) { if (pin == SD_CS) cs_high = value; }
void busy_wait_us(uint32_t us) { sim_us += us; }
void sleep_ms(uint32_t ms) { sim_us += (uint64_t)ms * 1000u; }
uint64_t time_us_64(void) { return sim_us; }
unsigned spi_init(int spi, unsigned b) { (void)spi; baud = b; return b; }
unsigned spi_set_baudrate(int spi, unsigned b) { (void)spi; baud = b; return b; }

int spi_write_read_blocking(int spi, const uint8_t *src, uint8_t *dst, size_t len)
{
    size_t i;

    (void)spi;
    for (i = 0; i < len; i++) {
        dst[i] = card_byte(src[i]);
        sim_us += 8000000u / baud + SPI_CALL_US;
    }
    return (int)len;
}

int spi_write_blocking(int spi, const uint8_t *src, size_t len)
{
    size_t i;

    (void)spi;
    for (i = 0; i < len; i++) {
        (void)card_byte(src[i]);
        sim_us += 8000000u / baud + SPI_CALL_US;
    }
    return (int)len;
}

/* ------------------------------------------------------------------ */
/*  The test                                                            */
/* ------------------------------------------------------------------ */
static void reset_card(void)
{
    memset(store, 0, sizeof store);
    sim_us = busy_until = 0;
    cs_high = true;
    card_ready = app_cmd = busy_after_response = false;
    acmd41_count = frame_len = out_head = out_tail = 0;
    state = S_CMD;
}

/* Write 100 blocks, each followed by a busy time of ms, then read them back. */
static void run(unsigned clock_hz, uint32_t ms)
{
    uint8_t block[512], back[512];
    unsigned i, k, write_fail = 0, read_fail = 0, wrong = 0;

    reset_card();
    busy_ms = ms;
    sd_init();
    if (sd_card_init() != SD_OK) {
        printf("%2u MHz %4u ms   the card did not start\n", clock_hz / 1000000u, (unsigned)ms);
        return;
    }
    spi_set_baudrate(SD_SPI, clock_hz);
    for (i = 0; i < 100; i++) {
        for (k = 0; k < 512; k++)
            block[k] = (uint8_t)(i * 7 + k);
        if (sd_write_block(i, block) != SD_OK)
            write_fail++;
    }
    busy_until = 0;                     /* let the card finish, then check what it holds */
    for (i = 0; i < 100; i++) {
        for (k = 0; k < 512; k++)
            block[k] = (uint8_t)(i * 7 + k);
        if (sd_read_block(i, back) != SD_OK)
            read_fail++;
        else if (memcmp(block, back, 512) != 0)
            wrong++;
    }
    printf("%2u MHz %4u ms   %3u of 100 writes failed, %3u reads failed, %3u blocks wrong\n",
           clock_hz / 1000000u, (unsigned)ms, write_fail, read_fail, wrong);
}

int main(void)
{
    static const uint32_t busy[] = { 1, 10, 20, 30, 40, 100, 300 };
    static const unsigned clocks[] = { 25000000u, 12000000u, 6000000u };
    unsigned c, b;

    printf("Fake SD card: busy for the stated time after every write.\n"
           "One SPI byte is assumed to take 8 bits at the clock plus %u us of call overhead.\n\n",
           (unsigned)SPI_CALL_US);
    for (c = 0; c < sizeof clocks / sizeof clocks[0]; c++) {
        for (b = 0; b < sizeof busy / sizeof busy[0]; b++)
            run(clocks[c], busy[b]);
        printf("\n");
    }
    return 0;
}
