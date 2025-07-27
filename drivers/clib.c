//
// clib.c - Interface to the C standard library functions for PicoCalc
//
// This file provides implementations for file operations using the FAT32 filesystem.
//
// Include this file in your project to enable file handling capabilities.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include "pico/stdlib.h"
#include "fat32.h"

#define FD_FLAG_MASK 0x4000 // Mask to indicate a file descriptor
#define MAX_OPEN_FILES 16

static int initialized = 0;
static fat32_file_t files[MAX_OPEN_FILES];

static void init(void)
{
    if (!initialized)
    {
        for (int i = 0; i < MAX_OPEN_FILES; i++)
        {
            files[i].is_open = 0;
        }
        initialized = 1;
    }
}

static int fat32_error_to_errno(fat32_error_t error)
{
    switch (error)
    {
    case FAT32_OK:
        return 0;
    case FAT32_ERROR_NO_CARD:
        return ENODEV;
    case FAT32_ERROR_NOT_MOUNTED:
        return ENODEV;
    case FAT32_ERROR_FILE_NOT_FOUND:
        return ENOENT;
    case FAT32_ERROR_INVALID_PATH:
        return ENAMETOOLONG;
    case FAT32_ERROR_NOT_A_DIRECTORY:
        return ENOTDIR;
    case FAT32_ERROR_NOT_A_FILE:
        return EFTYPE;
    case FAT32_ERROR_DIR_NOT_EMPTY:
        return ENOTEMPTY;
    case FAT32_ERROR_DIR_NOT_FOUND:
        return ENOENT;
    case FAT32_ERROR_DISK_FULL:
        return ENOSPC;
    case FAT32_ERROR_FILE_EXISTS:
        return EEXIST;
    case FAT32_ERROR_INVALID_POSITION:
        return ESPIPE;
    case FAT32_ERROR_INVALID_PARAMETER:
        return EINVAL;
    default:
        return EIO; // General I/O error for unknown errors
    }
}

int _open(const char *filename, int oflag, ...)
{
    fat32_error_t result;

    init(); // Ensure files are initialized

    for (int i = 0; i < MAX_OPEN_FILES; i++)
    {
        if (files[i].is_open == 0)
        {
            if ((result = fat32_open(&files[i], filename)) != FAT32_OK)
            {
                if (oflag & O_CREAT && result == FAT32_ERROR_FILE_NOT_FOUND)
                {
                    // If O_CREAT is set, try to create the file
                    if ((result = fat32_create(&files[i], filename)) != FAT32_OK)
                    {
                        errno = fat32_error_to_errno(result);
                        return -1; // Failed to create file
                    }
                }
                else
                {
                    errno = fat32_error_to_errno(result);
                    return -1; // Failed to open file
                }
            }
            else if (oflag & O_EXCL && files[i].is_open)
            {
                fat32_close(&files[i]); // Close the file if it already exists
                errno = EEXIST;              // File already exists and O_EXCL is set
                return -1;
            }

            if (oflag & O_TRUNC)
            {
                // If O_TRUNC is set, truncate the file
                files[i].file_size = 0;
                files[i].position = 0;
            }
            else if (oflag & O_APPEND)
            {
                // If O_APPEND is set, move to the end of the file
                files[i].position = files[i].file_size;
            }

            return i | FD_FLAG_MASK; // Return a file descriptor (positive value)
        }
    }
    errno = EMFILE; // Too many open files
    return -1;
}

int _close(int fd)
{
    fat32_error_t result;

    if ((fd & FD_FLAG_MASK) == 0)
    {
        errno = EBADF; // Invalid file descriptor
        return -1;
    }

    fd &= ~FD_FLAG_MASK; // Clear the file descriptor flag
    if (fd < 0 || fd >= MAX_OPEN_FILES || !files[fd].is_open)
    {
        errno = EBADF; // Invalid file descriptor
        return -1;
    }

    fat32_file_t *file = &files[fd];
    if ((result = fat32_close(file)) == FAT32_OK)
    {
        file->is_open = 0; // Mark as closed
        return 0;          // Success
    }

    errno = fat32_error_to_errno(result);
    return -1; // Failure
}

off_t _lseek(int fd, off_t offset, int whence)
{
    fat32_error_t result;

    if ((fd & FD_FLAG_MASK) == 0)
    {
        errno = EBADF; // Invalid file descriptor
        return -1;
    }

    fd &= ~FD_FLAG_MASK; // Clear the file descriptor flag

    if (fd < 0 || fd >= MAX_OPEN_FILES || !files[fd].is_open)
    {
        return -1; // Invalid file descriptor
    }

    fat32_file_t *file = &files[fd];

    if (whence == SEEK_SET)
    {
        file->position = offset;
    }
    else if (whence == SEEK_CUR)
    {
        file->position += offset;
    }
    else if (whence == SEEK_END)
    {
        file->position = file->file_size + offset;
    }

    if ((result = fat32_seek(file, offset)) == FAT32_OK)
    {
        return file->position; // Success
    }

    errno = fat32_error_to_errno(result);
    return -1; // Failure
}

int _read(int fd, char *buffer, int length)
{
    fat32_error_t result;

    if (fd == 0)
    {
        // Handle stdin
        return stdio_get_until(buffer, length, at_the_end_of_time);
    }

    if ((fd & FD_FLAG_MASK) == 0)
    {
        errno = EBADF; // Invalid file descriptor
        return -1;
    }

    fd &= ~FD_FLAG_MASK; // Clear the file descriptor flag

    if (fd < 0 || fd >= MAX_OPEN_FILES || !files[fd].is_open)
    {
        errno = EBADF; // Invalid file descriptor
        return -1;
    }

    fat32_file_t *file = &files[fd];
    size_t bytes_read = 0;
    if ((result = fat32_read(file, buffer, length, &bytes_read)) != FAT32_OK)
    {
        errno = fat32_error_to_errno(result);
        return -1; // Read failed
    }

    if (bytes_read > 0)
    {
        return bytes_read; // Return number of bytes read
    }

    return 0; // End of file
}

int _write(int fd, const char *buffer, int length)
{
    fat32_error_t result;

    if (fd == 1 || fd == 2)
    {
        // Handle stdout and stderr
        stdio_put_string(buffer, length, false, true);
        return length; // Return number of bytes written
    }

    if (length == 0)
    {
        return 0; // No data to write
    }

    if ((fd & FD_FLAG_MASK) == 0)
    {
        errno = EBADF; // Invalid file descriptor
        return -1;
    }

    fd &= ~FD_FLAG_MASK; // Clear the file descriptor flag

    if (fd < 0 || fd >= MAX_OPEN_FILES || !files[fd].is_open)
    {
        errno = EBADF; // Invalid file descriptor
        return -1;
    }

    fat32_file_t *file = &files[fd];
    size_t bytes_written = 0;
    if ((result = fat32_write(file, buffer, length, &bytes_written)) != FAT32_OK)
    {
        errno = fat32_error_to_errno(result);
        return -1; // Write failed
    }

    if (bytes_written > 0)
    {
        return bytes_written; // Return number of bytes written
    }

    errno = EIO; // Unexpected fallback, indicate I/O error
    return -1;
}

int _fstat(int fd, struct stat *buf)
{
    if ((fd & FD_FLAG_MASK) == 0)
    {
        errno = EBADF; // Invalid file descriptor
        return -1;
    }

    fd &= ~FD_FLAG_MASK; // Clear the file descriptor flag

    if (fd < 0 || fd >= MAX_OPEN_FILES || !files[fd].is_open)
    {
        errno = EBADF; // Invalid file descriptor
        return -1;
    }

    fat32_file_t *file = &files[fd];
    buf->st_size = file->file_size;

    if (file->attributes & FAT32_ATTR_DIRECTORY)
    {
        buf->st_mode = S_IFDIR | S_IRUSR | S_IWUSR | S_IXUSR; // Directory with rwx for user
    }
    else
    {
        buf->st_mode = S_IFREG | S_IRUSR; // Regular file, readable by user
        if (!(file->attributes & FAT32_ATTR_READ_ONLY))
        {
            buf->st_mode |= S_IWUSR; // Writable if not read-only
        }
    }
    buf->st_nlink = 1;
    buf->st_uid = 0;
    buf->st_gid = 0;
    buf->st_atime = 0;
    buf->st_mtime = 0;
    buf->st_ctime = 0;
    buf->st_ino = 0;
    return 0; // Success
}

int _stat(const char *path, struct stat *buf)
{
    int fd = _open(path, O_RDONLY);
    if (fd < 0) {
        // _open sets errno
        return -1;
    }

    int result = _fstat(fd, buf);
    _close(fd); // Always close, even if _fstat fails

    return result;
}

int _link(const char *oldpath, const char *newpath)
{
    return -1; // Linking is not supported in FAT32 :(
}

int _unlink(const char *filename)
{
    fat32_error_t result;

    if ((result = fat32_delete(filename)) != FAT32_OK)
    {
        errno = fat32_error_to_errno(result);
        return -1; // Deletion failed
    }
    return 0; // Success
}

int rename(const char *oldpath, const char *newpath)
{
    fat32_error_t result;

    if ((result = fat32_rename(oldpath, newpath)) != FAT32_OK)
    {
        errno = fat32_error_to_errno(result);
        return -1; // Rename failed
    }
    return 0; // Success
}