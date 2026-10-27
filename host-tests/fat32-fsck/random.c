/* random create / write / append / truncate / rename / delete, with the
   contents checked and fsck at the end of each round */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fat32.h"
void sd_open(const char *); void sd_close(void);
#define N 24
static size_t len[N]; static int fill[N]; static bool have[N];
static void name(int i, char *b) { snprintf(b, 96, "/d%d/file-with-a-long-name-%02d.txt", i % 3, i); }
static int fsck(const char *img, int round)
{
	char cmd[256]; snprintf(cmd, sizeof cmd, "fsck.fat -n %s > rfsck.out 2>&1", img);
	int rc = system(cmd);
	if (rc) { printf("round %d: fsck found problems\n", round); if (system("grep -v 'FATs differ\\|Using first\\|^fsck.fat\\|^Checking\\|^Leaving\\|^There are\\|Perform changes\\|^$' rfsck.out | head -8 | sed 's/^/    /'")) {} }
	return rc;
}
int main(int argc, char **argv)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	srand(argc > 2 ? atoi(argv[2]) : 1);
	sd_open(argv[1]); fat32_init(); fat32_mount();
	fat32_file_t d;
	for (int i = 0; i < 3; i++) { char p[8]; snprintf(p, 8, "/d%d", i); fat32_dir_create(&d, p); fat32_close(&d); }
	int bad = 0;
	static char buf[200000], rb[200000];
	for (int round = 0; round < 30; round++) {
		for (int op = 0; op < 60; op++) {
			int i = rand() % N; char p[96]; name(i, p);
			int what = rand() % 6;
			fat32_file_t f;
			if (!have[i] || what == 0) {		/* create (again) */
				if (have[i]) fat32_delete(p);
				if (fat32_create(&f, p) != FAT32_OK) { printf("create %s failed\n", p); bad++; continue; }
				len[i] = (rand() % 4) ? rand() % 100000 : 0; fill[i] = 'a' + rand() % 26;
				memset(buf, fill[i], len[i]); size_t w = 0;
				if (len[i]) fat32_write(&f, buf, len[i], &w);
				fat32_close(&f); have[i] = true;
			} else if (what == 1) {		/* append */
				if (fat32_open(&f, p) != FAT32_OK) { printf("open %s failed\n", p); bad++; continue; }
				size_t add = rand() % 70000; memset(buf, fill[i], add); size_t w;
				fat32_seek(&f, len[i]); fat32_write(&f, buf, add, &w); fat32_close(&f); len[i] += add;
			} else if (what == 2) {		/* delete */
				fat32_delete(p); have[i] = false;
			} else if (what == 3) {		/* rename it to a free slot */
				int j = rand() % N; if (have[j]) continue; char q[96]; name(j, q);
				if (fat32_rename(p, q) != FAT32_OK) { printf("rename failed\n"); bad++; continue; }
				have[j] = true; len[j] = len[i]; fill[j] = fill[i]; have[i] = false;
			} else {				/* read it back */
				if (fat32_open(&f, p) != FAT32_OK) { printf("open %s failed\n", p); bad++; continue; }
				size_t got = 0; fat32_read(&f, rb, sizeof rb, &got); fat32_close(&f);
				size_t want = len[i] < sizeof rb ? len[i] : sizeof rb;
				bool ok = got == want;
				for (size_t k = 0; ok && k < got; k++) ok = rb[k] == fill[i];
				if (!ok) { printf("round %d: %s reads wrong (%zu of %zu)\n", round, p, got, want); bad++; }
			}
		}
		sd_close();
		if (fsck(argv[1], round)) { bad++; break; }
		sd_open(argv[1]); fat32_unmount(); fat32_mount();
	}
	printf(bad ? "%d problems\n" : "30 rounds clean\n", bad);
	return bad != 0;
}
