/* clib.c's functions, renamed for the bench (see run.sh) so that they
   don't stand in for the PC's own C library */
#pragma once
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
int pc_open(const char *path, int oflag, ...);
int pc_close(int fd);
off_t pc_lseek(int fd, off_t offset, int whence);
int pc_read(int fd, char *buf, int len);
int pc_write(int fd, const char *buf, int len);
int pc_fstat(int fd, struct stat *st);
int pc_stat(const char *path, struct stat *st);
int pc_unlink(const char *path);
int pc_rename(const char *from, const char *to);
int pc_mkdir(const char *path, mode_t mode);

void sd_open(const char *path);
void sd_close(void);
void sd_set_present(bool present);


/* fsck.fat -n on the image; true if it found nothing. The card is taken
   out first and put back after, so FatFs mounts it afresh */
bool bench_fsck(const char *img, const char *out);
/* let fsck.fat fix the image, so the next step starts clean */
void bench_fix(const char *img);
