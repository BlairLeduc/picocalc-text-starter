/* The SD card, as a disk image file; and the rest of the Pico SDK that
   diskio.c and clib.c use */
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "sdcard.h"

static FILE *img;
static bool present = true;
static repeating_timer_t *detect;	/* diskio.c's card-detect timer */

void sd_open(const char *path) { img = fopen(path, "r+b"); if (!img) { perror(path); exit(1); } }
void sd_close(void) { if (img) fclose(img); img = NULL; }

/* take the card out, or put it back: as the 500 ms timer would see it */
void sd_set_present(bool p) { present = p; if (detect) detect->callback(detect); }

void sd_init(void) {}
bool sd_card_present(void) { return present && img; }
bool sd_is_sdhc(void) { return true; }
sd_error_t sd_card_init(void) { return present && img ? SD_OK : SD_ERROR_NO_CARD; }
sd_error_t sd_read_block(uint32_t b, uint8_t *buf)
{
	if (!present || !img || fseek(img, (long)b * 512, SEEK_SET) || fread(buf, 1, 512, img) != 512) return SD_ERROR_READ_FAILED;
	return SD_OK;
}
sd_error_t sd_write_block(uint32_t b, const uint8_t *buf)
{
	if (!present || !img || fseek(img, (long)b * 512, SEEK_SET) || fwrite(buf, 1, 512, img) != 512) return SD_ERROR_WRITE_FAILED;
	fflush(img);
	return SD_OK;
}
sd_error_t sd_read_blocks(uint32_t s, uint32_t n, uint8_t *buf) { for (uint32_t i = 0; i < n; i++) if (sd_read_block(s + i, buf + i * 512)) return SD_ERROR_READ_FAILED; return SD_OK; }
sd_error_t sd_write_blocks(uint32_t s, uint32_t n, const uint8_t *buf) { for (uint32_t i = 0; i < n; i++) if (sd_write_block(s + i, buf + i * 512)) return SD_ERROR_WRITE_FAILED; return SD_OK; }
const char *sd_error_string(sd_error_t e) { (void)e; return "?"; }

bool add_repeating_timer_ms(int32_t ms, repeating_timer_callback_t cb, void *user, repeating_timer_t *out)
{
	(void)ms; (void)user;
	out->callback = cb;
	detect = out;
	return true;
}
int stdio_get_until(char *buf, int len, absolute_time_t until) { (void)buf; (void)len; (void)until; return 0; }
void stdio_put_string(const char *s, int len, bool newline, bool cr) { (void)newline; (void)cr; fwrite(s, 1, (size_t)len, stdout); }
