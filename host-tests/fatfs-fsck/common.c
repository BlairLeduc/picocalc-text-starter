#include <stdio.h>
#include <stdlib.h>
#include "bench.h"

bool bench_fsck(const char *img, const char *out)
{
	sd_set_present(false);
	sd_close();
	char cmd[512];
	snprintf(cmd, sizeof cmd, "fsck.fat -n %s > %s 2>&1", img, out);
	int rc = system(cmd);
	sd_open(img);
	sd_set_present(true);
	return rc == 0;
}

void bench_fix(const char *img)
{
	sd_set_present(false);
	sd_close();
	char cmd[512];
	snprintf(cmd, sizeof cmd, "fsck.fat -a %s > /dev/null 2>&1", img);
	if (system(cmd)) {}
	sd_open(img);
	sd_set_present(true);
}
