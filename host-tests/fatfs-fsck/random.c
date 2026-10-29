/* Random creates, writes, appends, renames and deletes through clib.c and
   FatFs, with the contents checked and fsck.fat at the end of each round */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include "bench.h"
#include "filesystem.h"

#define N 24
static size_t len[N];
static int fill[N];
static bool have[N];

static void name(int i, char *b) { snprintf(b, 96, "/d%d/file-with-a-long-name-%02d.txt", i % 3, i); }

static bool write_all(int fd, const char *buf, size_t n)
{
	while (n) {
		int k = (int)(n < 16384 ? n : 16384);
		if (pc_write(fd, buf, k) != k) return false;
		buf += k; n -= (size_t)k;
	}
	return true;
}

int main(int argc, char **argv)
{
	setvbuf(stdout, NULL, _IONBF, 0);
	srand(argc > 2 ? (unsigned)atoi(argv[2]) : 1);
	sd_open(argv[1]);
	fs_init();
	for (int i = 0; i < 3; i++) { char p[8]; snprintf(p, 8, "/d%d", i); pc_mkdir(p, 0777); }
	int bad = 0;
	static char buf[200000], rb[200000];
	for (int round = 0; round < 30; round++) {
		for (int op = 0; op < 60; op++) {
			int i = rand() % N;
			char p[96];
			name(i, p);
			int what = rand() % 6;
			if (!have[i] || what == 0) {		/* create, or make it again ("w") */
				int fd = pc_open(p, O_WRONLY | O_CREAT | O_TRUNC);
				if (fd < 0) { printf("create %s failed: %s\n", p, strerror(errno)); bad++; continue; }
				len[i] = (rand() % 4) ? (size_t)(rand() % 100000) : 0;
				fill[i] = 'a' + rand() % 26;
				memset(buf, fill[i], len[i]);
				if (!write_all(fd, buf, len[i])) { printf("write %s failed\n", p); bad++; }
				pc_close(fd);
				have[i] = true;
			} else if (what == 1) {		/* append ("a") */
				int fd = pc_open(p, O_WRONLY | O_APPEND);
				if (fd < 0) { printf("open %s failed: %s\n", p, strerror(errno)); bad++; continue; }
				size_t add = (size_t)(rand() % 70000);
				memset(buf, fill[i], add);
				if (!write_all(fd, buf, add)) { printf("append %s failed\n", p); bad++; }
				pc_close(fd);
				len[i] += add;
			} else if (what == 2) {		/* delete */
				if (pc_unlink(p)) { printf("delete %s failed: %s\n", p, strerror(errno)); bad++; }
				have[i] = false;
			} else if (what == 3) {		/* rename it to a free name */
				int j = rand() % N;
				if (have[j]) continue;
				char q[96];
				name(j, q);
				if (pc_rename(p, q)) { printf("rename %s failed: %s\n", p, strerror(errno)); bad++; continue; }
				have[j] = true; len[j] = len[i]; fill[j] = fill[i]; have[i] = false;
			} else {				/* read it back */
				int fd = pc_open(p, O_RDONLY);
				if (fd < 0) { printf("open %s failed: %s\n", p, strerror(errno)); bad++; continue; }
				size_t got = 0;
				int k;
				while (got < sizeof rb && (k = pc_read(fd, rb + got, (int)(sizeof rb - got))) > 0) got += (size_t)k;
				pc_close(fd);
				size_t want = len[i] < sizeof rb ? len[i] : sizeof rb;
				bool ok = got == want;
				for (size_t n = 0; ok && n < got; n++) ok = rb[n] == fill[i];
				if (!ok) { printf("round %d: %s reads wrong (%zu of %zu)\n", round, p, got, want); bad++; }
			}
		}
		if (!bench_fsck(argv[1], "rfsck.out")) {
			printf("round %d: fsck found problems\n", round);
			if (system("grep -v '^fsck.fat\\|^Checking\\|^Leaving\\|Perform changes\\|^$' rfsck.out | head -8 | sed 's/^/    /'")) {}
			bad++;
			break;
		}
	}
	printf(bad ? "%d problems\n" : "30 rounds clean\n", bad);
	return bad != 0;
}
