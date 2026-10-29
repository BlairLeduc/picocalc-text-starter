/* What PicoCalc programs do to a card, a step at a time, through clib.c
   (the layer under fopen and friends) and FatFs, with fsck.fat looking
   after each step */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdint.h>
#include "bench.h"
#include "filesystem.h"

static const char *IMG;

/* a clock that says 2026-10-28 12:34:56 */
static bool clock_2026(uint16_t *date, uint16_t *time)
{
	return fs_pack_time(2026, 10, 28, 12, 34, 56, date, time);
}
static int bad;

static void check(const char *what)
{
	bool ok = bench_fsck(IMG, "fsck.out");
	printf("%-48s %s\n", what, ok ? "clean" : "PROBLEMS:");
	if (!ok) {
		bad++;
		if (system("grep -v '^fsck.fat\\|^Checking\\|^Leaving\\|Perform changes\\|^$' fsck.out | head -12 | sed 's/^/      /'")) {}
		bench_fix(IMG);
	}
}

/* the same, for a step where some damage can't be helped: allowed is what
   fsck.fat may say (and fix); anything else counts */
static void check_allowing(const char *what, const char *allowed)
{
	bool ok = bench_fsck(IMG, "fsck.out");
	if (ok) { printf("%-48s clean\n", what); return; }
	char cmd[512];
	snprintf(cmd, sizeof cmd, "grep -v '^fsck.fat\\|^Checking\\|^Leaving\\|Perform changes\\|^$\\|files, .* clusters$\\|%s' fsck.out > fsck.other", allowed);
	if (system(cmd)) {}
	FILE *f = fopen("fsck.other", "r");
	int c = f ? fgetc(f) : EOF;
	if (f) fclose(f);
	if (c == EOF) printf("%-48s as expected: %s\n", what, allowed);
	else {
		printf("%-48s PROBLEMS:\n", what);
		bad++;
		if (system("head -12 fsck.other | sed 's/^/      /'")) {}
	}
	remove("fsck.other");
	bench_fix(IMG);
}

static void fail(const char *what, const char *path)
{
	printf("  %s %s failed: %s\n", what, path, strerror(errno));
	bad++;
}

/* fopen(path, "w"), fwrite and fclose */
static void put(const char *path, size_t n, int fill)
{
	int fd = pc_open(path, O_WRONLY | O_CREAT | O_TRUNC);
	if (fd < 0) { fail("create", path); return; }
	static char buf[4096];
	memset(buf, fill, sizeof buf);
	while (n) {
		int k = (int)(n < sizeof buf ? n : sizeof buf);
		if (pc_write(fd, buf, k) != k) { fail("write", path); break; }
		n -= (size_t)k;
	}
	if (pc_close(fd)) fail("close", path);
}

/* the file holds n bytes of fill */
static bool holds(const char *path, size_t n, int fill)
{
	int fd = pc_open(path, O_RDONLY);
	if (fd < 0) return false;
	static char buf[4096];
	size_t got = 0;
	bool ok = true;
	int k;
	while ((k = pc_read(fd, buf, sizeof buf)) > 0) {
		for (int i = 0; i < k; i++) ok &= buf[i] == fill;
		got += (size_t)k;
	}
	pc_close(fd);
	return ok && k == 0 && got == n;
}

/* Diamond's save: the new version to name.tmp, the old one to name.bak,
   name.tmp into place */
static void diamond_save(const char *path, const char *bak, size_t n)
{
	char tmp[256];
	snprintf(tmp, sizeof tmp, "%s.tmp", path);
	pc_unlink(tmp);
	put(tmp, n, 'd');
	struct stat st;
	if (pc_stat(path, &st) == 0) {
		pc_unlink(bak);
		if (pc_rename(path, bak)) pc_unlink(path);
	}
	if (pc_rename(tmp, path)) fail("rename", tmp);
}

int main(int argc, char **argv)
{
	IMG = argv[1];
	setvbuf(stdout, NULL, _IONBF, 0);
	sd_open(IMG);
	fs_init();

	if (pc_mkdir("/documents", 0777)) fail("mkdir", "/documents");
	check("make a folder");

	put("/documents/first.md", 1000, 'a');
	check("one file");

	for (int i = 0; i < 40; i++) {			/* a folder with many long names */
		char p[128];
		snprintf(p, sizeof p, "/documents/a-rather-long-document-name-%02d.md", i);
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
	for (int i = 0; i < 40; i += 7) {
		char p[128];
		snprintf(p, sizeof p, "/documents/a-rather-long-document-name-%02d.md", i);
		if (!holds(p, 2000 + 19 * 500, 'd')) { printf("  %s doesn't hold the last save\n", p); bad++; }
	}

	for (int i = 0; i < 10; i++) {			/* screenshots, the way the programs take them */
		pc_mkdir("/screenshots", 0777);
		char p[64];
		snprintf(p, sizeof p, "/screenshots/btpair-%03d.txt", i + 1);
		put(p, 1400, 's');
		snprintf(p, sizeof p, "/screenshots/btpair-%03d.pklg", i + 1);
		put(p, 30000, 'k');
		snprintf(p, sizeof p, "/screenshots/diamond-%03d.bmp", i + 1);
		put(p, 204854, 'b');
	}
	check("screenshots");
	for (int i = 0; i < 10; i++) {
		char p[64];
		snprintf(p, sizeof p, "/screenshots/btpair-%03d.txt", i + 1); pc_unlink(p);
		snprintf(p, sizeof p, "/screenshots/btpair-%03d.pklg", i + 1); pc_unlink(p);
		snprintf(p, sizeof p, "/screenshots/diamond-%03d.bmp", i + 1); pc_unlink(p);
	}
	check("screenshots deleted");

	for (int i = 0; i < 5; i++) {			/* btpair's pairing file, saved in place */
		int fd = pc_open("/bluetooth/pairing.dat", O_RDWR);
		if (fd < 0) {
			pc_mkdir("/bluetooth", 0777);
			fd = pc_open("/bluetooth/pairing.dat", O_RDWR | O_CREAT);
			if (fd < 0) { fail("create", "/bluetooth/pairing.dat"); continue; }
		}
		static char st[1604];
		memset(st, i, sizeof st);
		pc_lseek(fd, 0, SEEK_SET);
		if (pc_write(fd, st, sizeof st) != (int)sizeof st) fail("write", "pairing.dat");
		pc_close(fd);
	}
	check("pairing file, saved five times");

	for (int i = 0; i < 5; i++) {			/* a settings file: deleted, then made again */
		pc_mkdir("/diamond", 0777);
		pc_unlink("/diamond/diamond.cfg");
		put("/diamond/diamond.cfg", 32, 'c');
	}
	check("settings file, remade five times");

	for (int i = 0; i < 5; i++) {			/* a log, appended to ("a") */
		int fd = pc_open("/logs/today.log", O_WRONLY | O_CREAT | O_APPEND);
		if (fd < 0) { pc_mkdir("/logs", 0777); fd = pc_open("/logs/today.log", O_WRONLY | O_CREAT | O_APPEND); }
		if (fd < 0) { fail("open", "/logs/today.log"); continue; }
		pc_lseek(fd, 0, SEEK_SET);		/* (appending goes on the end all the same) */
		if (pc_write(fd, "lllllllll\n", 10) != 10) fail("append", "/logs/today.log");
		pc_close(fd);
	}
	check("a log, appended to five times");
	if (!holds("/logs/today.log", 50, 'l')) {
		/* (the newlines aren't 'l': count them separately) */
		struct stat st;
		if (pc_stat("/logs/today.log", &st) || st.st_size != 50) { printf("  the log is %ld bytes, not 50\n", (long)st.st_size); bad++; }
	}

	put("/documents/empty.md", 0, 0);
	check("an empty file");
	{
		int fd = pc_open("/documents/empty.md", O_WRONLY);
		if (fd < 0 || pc_write(fd, "hello", 5) != 5) fail("write", "/documents/empty.md");
		if (fd >= 0) pc_close(fd);
	}
	check("writing to the empty file");
	{
		int fd = pc_open("/documents/empty.md", O_WRONLY | O_TRUNC);	/* emptied again */
		if (fd < 0) fail("truncate", "/documents/empty.md"); else pc_close(fd);
		struct stat st;
		if (pc_stat("/documents/empty.md", &st) || st.st_size != 0) { printf("  O_TRUNC left %ld bytes\n", (long)st.st_size); bad++; }
	}
	check("emptying it (O_TRUNC)");
	if (pc_unlink("/documents/empty.md")) fail("delete", "/documents/empty.md");
	check("deleting it");

	{						/* a file made longer by seeking past its end */
		int fd = pc_open("/documents/sparse.dat", O_RDWR | O_CREAT | O_TRUNC);
		if (fd < 0) fail("create", "/documents/sparse.dat");
		else {
			if (pc_lseek(fd, 100000, SEEK_SET) != 100000) fail("seek", "/documents/sparse.dat");
			if (pc_write(fd, "end", 3) != 3) fail("write", "/documents/sparse.dat");
			pc_close(fd);
		}
		struct stat st;
		if (pc_stat("/documents/sparse.dat", &st) || st.st_size != 100003) { printf("  sparse.dat is %ld bytes, not 100003\n", (long)st.st_size); bad++; }
	}
	check("a file made longer by seeking");

	{						/* a folder, emptied and deleted */
		pc_mkdir("/old", 0777);
		for (int i = 0; i < 30; i++) { char p[64]; snprintf(p, sizeof p, "/old/a-file-to-go-%02d.txt", i); put(p, 700, 'o'); }
		for (int i = 0; i < 30; i++) { char p[64]; snprintf(p, sizeof p, "/old/a-file-to-go-%02d.txt", i); pc_unlink(p); }
		if (pc_unlink("/old")) fail("delete", "/old");
	}
	check("a folder emptied and deleted");

	{						/* the card taken out with a file open */
		int fd = pc_open("/documents/open.md", O_WRONLY | O_CREAT | O_TRUNC);
		if (fd < 0) fail("create", "/documents/open.md");
		else {
			pc_write(fd, "before", 6);
			sd_set_present(false);
			sd_set_present(true);
			errno = 0;
			int w = pc_write(fd, "after", 5);
			if (w >= 0 || errno != EBADF) { printf("  a write to a file open before the card changed: %d, %s (should be EBADF)\n", w, strerror(errno)); bad++; }
			pc_close(fd);
			put("/documents/after.md", 100, 'n');	/* and the card works again */
			if (!holds("/documents/after.md", 100, 'n')) { printf("  after the card changed, a new file reads wrong\n"); bad++; }
		}
	}
	/* the file's first cluster is taken but its entry isn't written yet, so
	   that cluster is lost: as on any FAT card pulled with a file open */
	check_allowing("card out and in with a file open", "Reclaimed 1 unused cluster");

	{						/* dates: with no clock, and with one */
		struct stat st;
		put("/documents/undated.md", 10, 'u');
		if (pc_stat("/documents/undated.md", &st) == 0 && st.st_mtime == 0)
			printf("  (with no clock, files are dated 0: Windows shows no date for them)\n");
		fs_set_time_source(clock_2026);
		put("/documents/dated.md", 10, 'u');
		if (pc_stat("/documents/dated.md", &st) || st.st_mtime != 1793145600 + 12 * 3600 + 34 * 60 + 56)
			{ printf("  a file written with the clock at 2026-10-28 12:34:56 is dated %ld\n", (long)st.st_mtime); bad++; }
		fs_set_time_source(NULL);
	}
	check("dated files");

	{						/* things a program may count on */
		int a = pc_open("/documents/first.md", O_WRONLY);
		errno = 0;
		int b = pc_open("/documents/first.md", O_WRONLY);
		if (b >= 0 || errno != EBUSY) { printf("  a file opened twice to write: %d, %s (should be EBUSY)\n", b, strerror(errno)); bad++; }
		if (b >= 0) pc_close(b);
		if (a >= 0) pc_close(a);
		struct stat st;
		if (pc_stat("/", &st) || !S_ISDIR(st.st_mode)) { printf("  stat(\"/\") isn't a folder\n"); bad++; }
		errno = 0;
		if (pc_rename("/documents/first.md", "/documents/dated.md") == 0) printf("  (rename onto a file that's there replaces it)\n");
		else printf("  (rename onto a file that's there fails: %s; delete it first)\n", strerror(errno));
	}
	check("opening twice, stat, rename onto a file");

	printf(bad ? "%d steps with problems\n" : "all clean\n", bad);
	return bad != 0;
}
