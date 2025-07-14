#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "pico/stdlib.h"
#include "fat32.h"

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

int _open(const char *filename, int oflag, ...)
{
    init(); // Ensure files are initialized

    for (int i = 0; i < MAX_OPEN_FILES; i++)
    {
        if (files[i].is_open == 0)
        {
            if (fat32_file_open(&files[i], filename) != FAT32_OK)
            {
                if (oflag & O_CREAT)
                {
                    // If O_CREAT is set, try to create the file
                    if (fat32_file_create(&files[i], filename) != FAT32_OK)
                    {
                        return -1; // Failed to create file
                    }
                }
                else
                {
                    return -1; // Failed to open file
                }
                return -1; // Failed to open file
            } 

            if ((oflag & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL))
            {
                return -1; // O_CREAT and O_EXCL are set, but file already exists
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

            return i | 0x4000; // Return a file descriptor (positive value)
        }
    }
    return -1;
}

int _close(int fd)
{
    if (fd & 0x4000 == 0)
    {
        return -1; // Invalid file descriptor
    }

    fd &= ~0x4000; // Clear the file descriptor flag
    if (fd < 0 || fd >= MAX_OPEN_FILES || !files[fd].is_open)
    {
        return -1; // Invalid file descriptor
    }

    fat32_file_t *file = &files[fd];
    if (fat32_file_close(file) == FAT32_OK)
    {
        file->is_open = 0; // Mark as closed
        return 0;          // Success
    }

    return -1; // Failure
}

off_t _lseek(int fd, off_t offset, int whence)
{
    if (fd & 0x4000 == 0)
    {
        return -1; // Invalid file descriptor
    }

    fd &= ~0x4000; // Clear the file descriptor flag

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

    if (fat32_file_seek(file, offset) == FAT32_OK)
    {
        return file->position; // Success
    }

    return -1; // Failure
}

int _read(int fd, char *buffer, int length)
{
    if (fd == 0)
    {
        return stdio_get_until(buffer, length, at_the_end_of_time);
    }

    if (fd & 0x4000 == 0)
    {
        return -1; // Invalid file descriptor
    }

    fd &= ~0x4000; // Clear the file descriptor flag

    if (fd < 0 || fd >= MAX_OPEN_FILES || !files[fd].is_open)
    {
        return -1; // Invalid file descriptor
    }

    fat32_file_t *file = &files[fd];
    size_t bytes_read = 0;
    if (fat32_file_read(file, buffer, length, &bytes_read) != FAT32_OK)
    {
        return -1; // Read failed
    }

    if (bytes_read > 0)
    {
        return bytes_read; // Return number of bytes read
    }

    return -1; // Failure
}

int _write(int fd, const char *buffer, int length)
{
    if (fd == 1 || fd == 2)
    {
        stdio_put_string(buffer, length, false, true);
        return length; // Return number of bytes written
    }

    if (fd & 0x4000 == 0)
    {
        return -1; // Invalid file descriptor
    }

    fd &= ~0x4000; // Clear the file descriptor flag

    if (fd < 0 || fd >= MAX_OPEN_FILES || !files[fd].is_open)
    {
        return -1; // Invalid file descriptor
    }

    fat32_file_t *file = &files[fd];
    size_t bytes_written = 0;
    if (fat32_file_write(file, buffer, length, &bytes_written) != FAT32_OK)
    {
        return -1; // Write failed
    }

    if (bytes_written > 0)
    {
        return bytes_written; // Return number of bytes written
    }

    return -1; // Failure
}

int _fstat(int fd, struct stat *buf)
{
    if (fd & 0x4000 == 0)
    {
        return -1; // Invalid file descriptor
    }

    fd &= ~0x4000; // Clear the file descriptor flag

    if (fd < 0 || fd >= MAX_OPEN_FILES || !files[fd].is_open)
    {
        return -1; // Invalid file descriptor
    }

    fat32_file_t *file = &files[fd];
    buf->st_size = file->file_size;
    buf->st_mode = S_IFREG | S_IRUSR | S_IWUSR;
    buf->st_nlink = 1;
    buf->st_uid = 0;
    buf->st_gid = 0;
    buf->st_atime = 0;
    buf->st_mtime = 0;
    buf->st_ctime = 0;
    buf->st_ino = 0;
    return 0; // Success
}