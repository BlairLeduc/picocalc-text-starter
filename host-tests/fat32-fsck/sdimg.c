/* the SD card, as a disk image file */
#include <stdio.h>
#include <stdlib.h>
#include "sdcard.h"
static FILE *img;
long sd_writes;
void sd_open(const char *path) { img = fopen(path, "r+b"); if (!img) { perror(path); exit(1); } }
void sd_close(void) { if (img) fclose(img); img = NULL; }
void sd_init(void) {}
bool sd_card_present(void) { return true; }
bool sd_is_sdhc(void) { return true; }
sd_error_t sd_card_init(void) { return SD_OK; }
sd_error_t sd_read_block(uint32_t b, uint8_t *buf) { fseek(img, (long)b * 512, SEEK_SET); if (fread(buf, 1, 512, img) != 512) return 1; return SD_OK; }
sd_error_t sd_write_block(uint32_t b, const uint8_t *buf) { sd_writes++; fseek(img, (long)b * 512, SEEK_SET); fwrite(buf, 1, 512, img); fflush(img); return SD_OK; }
sd_error_t sd_read_blocks(uint32_t s, uint32_t n, uint8_t *buf) { for (uint32_t i = 0; i < n; i++) if (sd_read_block(s + i, buf + i * 512)) return 1; return SD_OK; }
sd_error_t sd_write_blocks(uint32_t s, uint32_t n, const uint8_t *buf) { for (uint32_t i = 0; i < n; i++) sd_write_block(s + i, buf + i * 512); return SD_OK; }
const char *sd_error_string(sd_error_t e) { (void)e; return "?"; }
