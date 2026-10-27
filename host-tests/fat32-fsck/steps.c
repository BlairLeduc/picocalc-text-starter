/* the PicoCalc programs' ways with the card, against a disk image, with
   fsck.fat looking after each step */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "fat32.h"
void sd_open(const char *); void sd_close(void);
static const char *IMG;
static int bad;

static void check(const char *what)
{
	sd_close();
	char cmd[512];
	snprintf(cmd, sizeof cmd, "fsck.fat -n %s > fsck.out 2>&1; echo $? > fsck.rc", IMG);
	if (system(cmd)) {}
	FILE *f = fopen("fsck.rc", "r"); int rc = 1; if (fscanf(f, "%d", &rc) != 1) rc = 1; fclose(f);
	printf("%-48s %s\n", what, rc == 0 ? "clean" : "PROBLEMS:");
	if (rc) { bad++; if (system("grep -v 'FATs differ\\|Using first FAT\\|^fsck.fat\\|^Checking\\|^Leaving\\|^There are differences\\|^ *[0-9]*:\\|Perform changes\\|^$' fsck.out | head -12 | sed 's/^/      /'")) {} }
	/* a clean slate for the next step: let fsck fix it */
	snprintf(cmd, sizeof cmd, "fsck.fat -a %s > /dev/null 2>&1", IMG);
	if (system(cmd)) {}
	sd_open(IMG);
	fat32_unmount(); fat32_mount();
}

static void put(const char *path, size_t n, int fill)
{
	fat32_file_t f;
	if (fat32_create(&f, path) != FAT32_OK) { printf("  create %s failed\n", path); return; }
	static char buf[4096]; memset(buf, fill, sizeof buf);
	while (n) { size_t k = n < sizeof buf ? n : sizeof buf, w; fat32_write(&f, buf, k, &w); n -= k; }
	fat32_close(&f);
}

/* Diamond's ed_io_save */
static void diamond_save(const char *path, const char *bak, size_t n)
{
	char tmp[256]; snprintf(tmp, sizeof tmp, "%s.tmp", path);
	fat32_delete(tmp);
	put(tmp, n, 'd');
	fat32_file_t old;
	if (fat32_open(&old, path) == FAT32_OK) {
		fat32_close(&old);
		fat32_delete(bak);
		if (fat32_rename(path, bak) != FAT32_OK) fat32_delete(path);
	}
	if (fat32_rename(tmp, path) != FAT32_OK) printf("  rename failed\n");
}

int main(int argc, char **argv)
{
	IMG = argv[1];
	setvbuf(stdout, NULL, _IONBF, 0);
	sd_open(IMG);
	fat32_init();
	if (fat32_mount() != FAT32_OK) { printf("mount failed\n"); return 1; }
	fat32_file_t d;
	fat32_dir_create(&d, "/documents"); fat32_close(&d);
	check("make a folder");

	put("/documents/first.md", 1000, 'a');
	check("one file");

	for (int i = 0; i < 40; i++) {			/* a folder with many long names */
		char p[128]; snprintf(p, sizeof p, "/documents/a-rather-long-document-name-%02d.md", i);
		put(p, 3000 + i * 100, 'x');
	}
	check("40 files with long names");

	for (int r = 0; r < 20; r++)
		for (int i = 0; i < 40; i += 7) {
			char p[128], b[128];
			snprintf(p, sizeof p, "/documents/a-rather-long-document-name-%02d.md", i);
			snprintf(b, sizeof b, "/documents/a-rather-long-document-name-%02d.bak", i);
			diamond_save(p, b, 2000 + r * 500);
		}
	check("Diamond saves, many times");

	for (int i = 0; i < 10; i++) {			/* screenshots, the way the programs take them */
		if (fat32_dir_create(&d, "/screenshots") == FAT32_OK) fat32_close(&d);
		char p[64]; snprintf(p, sizeof p, "/screenshots/btpair-%03d.txt", i + 1);
		put(p, 1400, 's');
		snprintf(p, sizeof p, "/screenshots/diamond-%03d.bmp", i + 1);
		put(p, 204854, 'b');
	}
	check("screenshots");
	for (int i = 0; i < 10; i++) {
		char p[64]; snprintf(p, sizeof p, "/screenshots/btpair-%03d.txt", i + 1); fat32_delete(p);
		snprintf(p, sizeof p, "/screenshots/diamond-%03d.bmp", i + 1); fat32_delete(p);
	}
	check("screenshots deleted");

	for (int i = 0; i < 5; i++) {			/* btpair's pairing file, saved in place */
		fat32_file_t f;
		if (fat32_open(&f, "/bluetooth/pairing.dat") != FAT32_OK) {
			if (fat32_dir_create(&d, "/bluetooth") == FAT32_OK) fat32_close(&d);
			if (fat32_create(&f, "/bluetooth/pairing.dat") != FAT32_OK) { printf("  pairing create failed\n"); continue; }
		}
		static char st[1604]; memset(st, i, sizeof st); size_t w;
		fat32_seek(&f, 0); fat32_write(&f, st, sizeof st, &w); fat32_close(&f);
	}
	check("pairing file, saved five times");

	for (int i = 0; i < 5; i++) {			/* a settings file: deleted, then made again */
		if (fat32_dir_create(&d, "/diamond") == FAT32_OK) fat32_close(&d);
		fat32_delete("/diamond/diamond.cfg");
		put("/diamond/diamond.cfg", 32, 'c');
	}
	check("settings file, remade five times");

	put("/documents/empty.md", 0, 0);
	check("an empty file");
	{ fat32_file_t f; size_t w; if (fat32_open(&f, "/documents/empty.md") == FAT32_OK) { fat32_write(&f, "hello", 5, &w); fat32_close(&f); } }
	check("writing to the empty file");

	fat32_delete("/documents/empty.md");
	check("deleting it");

	printf(bad ? "%d steps with problems\n" : "all clean\n", bad);
	return 0;
}
