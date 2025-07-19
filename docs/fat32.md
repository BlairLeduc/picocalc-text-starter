# FAT32

The FAT32 driver implements the FAT32 file system commonly used on SD cards. This driver does not support FAT 12 or FAT 16 file systems. One draw back to this is that cards 4 GB or less may be formated with FAT 16 by default, which this driver will be unable to read.

This driver is designed to be used with the [SD Card](docs/sdcard.md) driver, which provides the low-level access to the SD card. The FAT32 driver uses the SD Card driver to read and write sectors on the SD card.

## fat32_is_ready

`bool fat32_is_ready(void)`

Returns true if the SD card is initialised, mounted, and ready to use.


## fat32_mount

`fat32_error_t fat32_mount(void)`

Mount the SD card for use with the file/directory access functions.

Returns FAT32_OK if successful, otherwise an error code is returned.


## fat32_unmount

`void fat32_unmount(void)`

Unmounts the SD card.


## fat32_is_mounted

`bool fat32_is_mounted(void)`

Returns true if the SD card is mounted.


## fat32_get_status

`fat32_error_t fat32_get_status(void)`

Returns the status of the last mount operation.


## fat32_get_free_space

`fat32_error_t fat32_get_free_space(uint64_t *free_space)`

Returns the free space on the SD card. If available, the estimated free space is returned, otherwise the free space is computed, which may take many seconds.

Returns FAT32_OK if successful, otherwise an error code is returned.

### Parameters

- free_space – the target to store the the 64-bit value


## fat32_get_total_space

`fat32_error_t fat32_get_total_space(uint64_t *total_space)`

Returns the usable size of the SD card.

Returns FAT32_OK if successful, otherwise an error code is returned.

### Parameters

- total_space – the target to store the the 64-bit value


## fat32_get_volume_name

`fat32_error_t fat32_get_volume_name(char *name, size_t name_len)`

Gets the volumne name. 

Returns FAT32_OK if successful, otherwise an error code is returned.

### Parameters

- name – the buffer used to store the volume name
- name_len – the size of the provided buffer


## fat32_open

`fat32_error_t fat32_open(fat32_file_t *file, const char *path)`

Opens a file specified by the path and populates a `fat32-File_t`.

Returns FAT32_OK if successful, otherwise an error code is returned.

### Parameters

- file - the `fat32_file_t` representing the open file
- path – the path to the file to open


## fat32_create

`fat32_error_t fat32_create(fat32_file_t *file, const char *path)`

Creates a new file specified by the pathname and populates a `fat32_file_t`.

Returns FAT32_OK if successful, otherwise an error code is returned.

### Parameters

- file - the `fat32_file_t` representing the new file
- path – the path to the file to create


## fat32_close

`fat32_error_t fat32_close(fat32_file_t *file)`

Close the open file.

Returns FAT32_OK if successful, otherwise an error code is returned.

### Parameters

- file - the `fat32_file_t` representing the open file


## fat32_read

`fat32_error_t fat32_read(fat32_file_t *file, void *buffer, size_t size, size_t *bytes_read)`

Populates a buffer with the requested number of bytes from the opened file.

Returns FAT32_OK if successful, otherwise an error code is returned.

### Parameters

- file - the `fat32_file_t` representing the open file
- buffer – the buffer used to store the requested data (must be at east size in length)
- size - the number of bytes requested
- bytes_read – the number of bytes copied into the buffer


## fat32_write

`fat32_error_t fat32_write(fat32_file_t *file, const void *buffer, size_t size)`

Writes data to the opened file from the provided buffer.

Returns FAT32_OK if successful, otherwise an error code is returned.

### Parameters

- file - the `fat32_file_t` representing the open file
- buffer – the buffer containing the data to write (must be at least size in length)
- size - the number of bytes to write from the buffer


## fat32_seek

`fat32_error_t fat32_seek(fat32_file_t *file, uint32_t position)`

Sets the current position in the file to the specified position.

Returns FAT32_OK if successful, otherwise an error code is returned.

### Parameters

- file - the `fat32_file_t` representing the open file
- position – the position in the file to seek to (0 is the start of the file)


## fat32_tell

`uint32_t fat32_tell(fat32_file_t *file)`

Returns the current position in the file.

### Parameters

- file - the `fat32_file_t` representing the open file


## fat32_size

`uint32_t fat32_size(fat32_file_t *file)`

Returns the size of the file in bytes.

### Parameters

- file - the `fat32_file_t` representing the open file


## fat32_eof

`bool fat32_eof(fat32_file_t *file)`

Returns true if the end of the file has been reached.

### Parameters

- file - the `fat32_file_t` representing the open file


## fat32_delete

`fat32_error_t fat32_delete(const char *path)`

Deletes a file specified by the path.

Returns FAT32_OK if successful, otherwise an error code is returned.

### Parameters

- path – the path to the file to delete


## fat32_set_current_dir

`fat32_error_t fat32_set_current_dir(const char *path)`

Sets the current directory to the specified path.

Returns FAT32_OK if successful, otherwise an error code is returned.

### Parameters

- path – the path to the directory to set as current


## fat32_get_current_dir

`fat32_error_t fat32_get_current_dir(char *buffer, size_t buffer_size)`

Gets the current directory path.

Returns FAT32_OK if successful, otherwise an error code is returned.

### Parameters

- buffer – the buffer used to store the current directory path
- buffer_size – the size of the provided buffer


## fat32_dir_read

`fat32_error_t fat32_dir_read(fat32_file_t *dir, fat32_dir_entry_t *entry)`

Reads the next entry in the opened directory.

Returns FAT32_OK if successful, otherwise an error code is returned.

### Parameters

- dir - the `fat32_file_t` representing the open directory
- entry – the `fat32_dir_entry_t` to populate with the directory entry data


## fat32_dir_create

`fat32_error_t fat32_dir_create(fat32_file_t *dir, const char *path)`

Creates a new directory specified by the path and populates a `fat32_file_t`.

Returns FAT32_OK if successful, otherwise an error code is returned.

### Parameters

- dir - the `fat32_file_t` representing the new directory
- path – the path to the directory to create


## fat32_error_string

`const char *fat32_error_string(fat32_error_t error)`

Returns a string representation of the FAT32 error code.
