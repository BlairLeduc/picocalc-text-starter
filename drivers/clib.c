//
// clib.c - Interface to the C standard library functions for PicoCalc
//
// This file provides implementations for file operations using FatFs.
//
// Include this file in your project to enable file handling capabilities.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include "pico/stdlib.h"
#include "filesystem.h"

#define FD_FLAG_MASK 0x4000 // Mask to indicate a file descriptor
#define MAX_OPEN_FILES 16

static FIL files[MAX_OPEN_FILES];
static bool in_use[MAX_OPEN_FILES];
static bool append[MAX_OPEN_FILES]; // O_APPEND: every write goes on the end

static int fresult_to_errno(FRESULT result)
{
    switch (result)
    {
    case FR_OK:
        return 0;
    case FR_NOT_READY:
    case FR_INVALID_DRIVE:
    case FR_NOT_ENABLED:
    case FR_NO_FILESYSTEM:
        return ENODEV;
    case FR_NO_FILE:
    case FR_NO_PATH:
        return ENOENT;
    case FR_INVALID_NAME:
        return EINVAL;
    case FR_DENIED:
        return EACCES;
    case FR_EXIST:
        return EEXIST;
    case FR_INVALID_OBJECT:
        return EBADF;
    case FR_WRITE_PROTECTED:
        return EROFS;
    case FR_LOCKED:
        return EBUSY;
    case FR_NOT_ENOUGH_CORE:
        return ENOMEM;
    case FR_TOO_MANY_OPEN_FILES:
        return EMFILE;
    case FR_INVALID_PARAMETER:
        return EINVAL;
    default:
        return EIO; // General I/O error for unknown errors
    }
}

// The open file for a file descriptor, or NULL (and errno set)
static FIL *get_file(int fd)
{
    if ((fd & FD_FLAG_MASK) == 0)
    {
        errno = EBADF; // Invalid file descriptor
        return NULL;
    }

    fd &= ~FD_FLAG_MASK; // Clear the file descriptor flag
    if (fd < 0 || fd >= MAX_OPEN_FILES || !in_use[fd])
    {
        errno = EBADF; // Invalid file descriptor
        return NULL;
    }

    return &files[fd];
}

// A FAT date and time as a time_t, taking them to be UTC. (mktime() would
// do, but it brings in newlib's time zone and locale code, some 25 KB.)
static time_t fat_to_time(WORD fdate, WORD ftime)
{
    if (fdate == 0)
    {
        return 0; // Never dated
    }

    static const uint16_t days_before_month[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    int year = (fdate >> 9) + 1980;
    int month = (fdate >> 5) & 15;
    int day = fdate & 31;
    if (month < 1 || month > 12)
    {
        return 0;
    }

    // Days from 1970 to this year, counting leap days. In FAT's 1980-2107,
    // every fourth year is a leap year but 2100.
    int32_t days = (year - 1970) * 365 + (year - 1969) / 4 - (year > 2100);
    days += days_before_month[month - 1] + day - 1;
    if (month > 2 && year % 4 == 0 && year != 2100)
    {
        days++; // This year's 29 February
    }

    return (time_t)days * 86400 + (ftime >> 11) * 3600 + ((ftime >> 5) & 63) * 60 + (ftime & 31) * 2;
}

int _open(const char *filename, int oflag, ...)
{
    BYTE mode;
    FRESULT result;

    switch (oflag & O_ACCMODE)
    {
    case O_WRONLY:
        mode = FA_WRITE;
        break;
    case O_RDWR:
        mode = FA_READ | FA_WRITE;
        break;
    default:
        mode = FA_READ;
        break;
    }

    if ((oflag & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL))
    {
        mode |= FA_CREATE_NEW; // Fail if it exists
    }
    else if ((oflag & (O_CREAT | O_TRUNC)) == (O_CREAT | O_TRUNC))
    {
        mode |= FA_CREATE_ALWAYS; // Create, or empty if it exists
    }
    else if (oflag & O_CREAT)
    {
        mode |= FA_OPEN_ALWAYS; // Create if it doesn't exist
    }

    for (int i = 0; i < MAX_OPEN_FILES; i++)
    {
        if (!in_use[i])
        {
            if ((result = f_open(&files[i], filename, mode)) != FR_OK)
            {
                errno = fresult_to_errno(result);
                return -1; // Failed to open file
            }

            if ((oflag & (O_CREAT | O_TRUNC)) == O_TRUNC && (mode & FA_WRITE))
            {
                // O_TRUNC without O_CREAT: empty the file that is there
                if ((result = f_truncate(&files[i])) != FR_OK)
                {
                    f_close(&files[i]);
                    errno = fresult_to_errno(result);
                    return -1;
                }
            }

            in_use[i] = true;
            append[i] = (oflag & O_APPEND) != 0;
            return i | FD_FLAG_MASK; // Return a file descriptor (positive value)
        }
    }
    errno = EMFILE; // Too many open files
    return -1;
}

int _close(int fd)
{
    FIL *file = get_file(fd);
    if (file == NULL)
    {
        return -1;
    }

    // The slot is free even if the close fails (say the card was pulled)
    in_use[fd & ~FD_FLAG_MASK] = false;

    FRESULT result = f_close(file);
    if (result != FR_OK)
    {
        errno = fresult_to_errno(result);
        return -1; // Failure
    }
    return 0; // Success
}

off_t _lseek(int fd, off_t offset, int whence)
{
    FIL *file = get_file(fd);
    if (file == NULL)
    {
        return -1;
    }

    FSIZE_t position;
    if (whence == SEEK_SET)
    {
        position = offset;
    }
    else if (whence == SEEK_CUR)
    {
        position = f_tell(file) + offset;
    }
    else if (whence == SEEK_END)
    {
        position = f_size(file) + offset;
    }
    else
    {
        errno = EINVAL;
        return -1;
    }

    // Seeking past the end of a file opened for reading only stops at the
    // end; opened for writing, it makes the file longer.
    FRESULT result = f_lseek(file, position);
    if (result != FR_OK)
    {
        errno = fresult_to_errno(result);
        return -1; // Failure
    }
    return f_tell(file); // Success
}

int _read(int fd, char *buffer, int length)
{
    if (fd == 0)
    {
        // Handle stdin
        return stdio_get_until(buffer, length, at_the_end_of_time);
    }

    FIL *file = get_file(fd);
    if (file == NULL)
    {
        return -1;
    }

    UINT bytes_read = 0;
    FRESULT result = f_read(file, buffer, length, &bytes_read);
    if (result != FR_OK)
    {
        errno = fresult_to_errno(result);
        return -1; // Read failed
    }
    return bytes_read; // 0 at end of file
}

int _write(int fd, const char *buffer, int length)
{
    if (fd == 1 || fd == 2)
    {
        // Handle stdout and stderr
        stdio_put_string(buffer, length, false, true);
        return length; // Return number of bytes written
    }

    FIL *file = get_file(fd);
    if (file == NULL)
    {
        return -1;
    }

    if (length == 0)
    {
        return 0; // No data to write
    }

    FRESULT result;
    if (append[fd & ~FD_FLAG_MASK] && (result = f_lseek(file, f_size(file))) != FR_OK)
    {
        errno = fresult_to_errno(result);
        return -1;
    }

    UINT bytes_written = 0;
    if ((result = f_write(file, buffer, length, &bytes_written)) != FR_OK)
    {
        errno = fresult_to_errno(result);
        return -1; // Write failed
    }

    if (bytes_written == 0)
    {
        errno = ENOSPC; // FatFs writes short when the card is full
        return -1;
    }
    return bytes_written; // Return number of bytes written
}

int _fstat(int fd, struct stat *buf)
{
    FIL *file = get_file(fd);
    if (file == NULL)
    {
        return -1;
    }

    memset(buf, 0, sizeof(*buf));
    buf->st_size = f_size(file);
    buf->st_mode = S_IFREG | S_IRUSR; // Regular file, readable by user
    if (!(file->obj.attr & AM_RDO))
    {
        buf->st_mode |= S_IWUSR; // Writable if not read-only
    }
    buf->st_nlink = 1;
    return 0; // Success
}

int _stat(const char *path, struct stat *buf)
{
    FILINFO info;
    FRESULT result = f_stat(path, &info);

    if (result == FR_INVALID_NAME)
    {
        // f_stat can't describe the root directory (or "." at the root),
        // so see if the path opens as a directory
        DIR dir;
        if (f_opendir(&dir, path) == FR_OK)
        {
            f_closedir(&dir);
            memset(buf, 0, sizeof(*buf));
            buf->st_mode = S_IFDIR | S_IRUSR | S_IWUSR | S_IXUSR;
            buf->st_nlink = 1;
            return 0;
        }
    }

    if (result != FR_OK)
    {
        errno = fresult_to_errno(result);
        return -1;
    }

    memset(buf, 0, sizeof(*buf));
    buf->st_size = info.fsize;
    if (info.fattrib & AM_DIR)
    {
        buf->st_mode = S_IFDIR | S_IRUSR | S_IWUSR | S_IXUSR; // Directory with rwx for user
    }
    else
    {
        buf->st_mode = S_IFREG | S_IRUSR; // Regular file, readable by user
        if (!(info.fattrib & AM_RDO))
        {
            buf->st_mode |= S_IWUSR; // Writable if not read-only
        }
    }
    buf->st_nlink = 1;
    buf->st_mtime = fat_to_time(info.fdate, info.ftime);
    return 0; // Success
}

int _link(const char *oldpath, const char *newpath)
{
    errno = EMLINK;
    return -1; // Linking is not supported in FAT :(
}

int _unlink(const char *filename)
{
    FRESULT result = f_unlink(filename);
    if (result != FR_OK)
    {
        errno = fresult_to_errno(result);
        return -1; // Deletion failed
    }
    return 0; // Success
}

int rename(const char *oldpath, const char *newpath)
{
    FRESULT result = f_rename(oldpath, newpath);
    if (result != FR_OK)
    {
        errno = fresult_to_errno(result);
        return -1; // Rename failed
    }
    return 0; // Success
}

int mkdir(const char *path, mode_t mode)
{
    FRESULT result = f_mkdir(path);
    if (result != FR_OK)
    {
        errno = fresult_to_errno(result);
        return -1;
    }
    return 0;
}
