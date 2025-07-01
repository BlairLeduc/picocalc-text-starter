# SD Card Driver Documentation

This document describes the SD card driver implementation for the PicoCalc with FAT32 support.

## Overview

The SD card driver provides:
- Block-level read/write access to SD cards
- Basic FAT32 file system support
- SPI interface using SPI0
- Simple file operations (read-only in current implementation)

## Hardware Connections

The SD card is connected to the following GPIO pins on the Raspberry Pi Pico:

| Function | GPIO Pin | Description |
|----------|----------|-------------|
| MISO     | 16       | Master In, Slave Out |
| CS       | 17       | Chip Select |
| SCK      | 18       | Serial Clock |
| MOSI     | 19       | Master Out, Slave In |
| CD       | 22       | Card Detect (active low) |

## Available Commands

The following commands are available in the PicoCalc text interface:

### `sd_status`
Shows the current status of the SD card:
- Card detection
- Initialization status
- File system type (FAT12/FAT16/FAT32)
- Total storage space

### `sd_mount`
Attempts to mount the SD card and initialize the FAT32 file system.

### `sd_list`
Lists files on the SD card (placeholder - not fully implemented).

### `sd_read <filename>`
Reads and displays the contents of a text file from the SD card.
Currently only supports files in the root directory.

## Usage Example

```
sd_status        # Check if SD card is present and initialized
sd_mount         # Mount the SD card file system
sd_read readme.txt  # Read and display readme.txt
```

## Implementation Notes

### Current Limitations
- **Read-only**: File writing and creation are not implemented
- **Root directory only**: Subdirectory navigation is not supported
- **Short filenames**: Only 8.3 format filenames are supported
- **Simplified FAT**: FAT cluster chain following is simplified
- **Single file**: Only one file can be open at a time

### Technical Details
- Uses SPI0 interface at 25 MHz
- Supports SD cards formatted with FAT32
- 512-byte sector size
- Thread-safe with semaphore protection
- Compatible with SDHC cards

### File System Support
The driver currently supports:
- FAT32 file systems (primary target)
- Basic FAT16 recognition
- Boot sector validation
- Root directory entry parsing

## API Reference

### Initialization Functions
```c
sd_error_t sd_init(void);           // Initialize SD card
sd_error_t sd_mount(void);          // Mount file system
void sd_deinit(void);               // Cleanup
```

### File Operations
```c
sd_error_t sd_file_open(sd_file_t *file, const char *filename);
sd_error_t sd_file_read(sd_file_t *file, void *buffer, size_t size, size_t *bytes_read);
sd_error_t sd_file_close(sd_file_t *file);
```

### Block-level Operations
```c
sd_error_t sd_read_block(uint32_t block, uint8_t *buffer);
sd_error_t sd_write_block(uint32_t block, const uint8_t *buffer);
```

## Error Handling

The driver returns `sd_error_t` codes:
- `SD_OK`: Success
- `SD_ERROR_NO_CARD`: No SD card present
- `SD_ERROR_INIT_FAILED`: Initialization failed
- `SD_ERROR_NOT_MOUNTED`: File system not mounted
- `SD_ERROR_FILE_NOT_FOUND`: Requested file not found

Use `sd_error_string(error)` to get human-readable error messages.

## Future Enhancements

Planned improvements include:
- File writing and creation
- Directory navigation
- Long filename support
- Multiple file handles
- FAT cluster chain management
- Directory listing
- File deletion

## Testing

To test the SD card functionality:
1. Format an SD card with FAT32
2. Create a file called `readme.txt` in the root directory
3. Insert the SD card into the PicoCalc
4. Use the commands above to test functionality

The driver has been tested with:
- Standard SD cards (up to 2GB)
- SDHC cards (4GB-32GB)
- Various FAT32 formatters
