#ifndef SDCARD_H
#define SDCARD_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define SD_SPI          (spi0)

// Raspberry Pi Pico board GPIO pins
#define SD_MISO         (16)             // master in, slave out (MISO)
#define SD_CS           (17)             // chip select (CS)
#define SD_SCK          (18)             // serial clock (SCK)
#define SD_MOSI         (19)             // master out, slave in (MOSI)
#define SD_DETECT       (22)             // card detect (CD)

// SD card interface definitions
#define SD_BAUDRATE     (25000000)       // 25 MHz SPI clock speed (SD spec max for SPI mode)

// Card type detection (after successful initialization)
typedef enum {
    CARD_TYPE_UNKNOWN = 0,
    CARD_TYPE_SD_V1,      // SD Version 1.x (up to 2GB)
    CARD_TYPE_SD_V2,      // SD Version 2.x Standard Capacity (up to 2GB)
    CARD_TYPE_SDHC,       // SD High Capacity (4GB to 32GB)
    CARD_TYPE_SDXC        // SD eXtended Capacity (64GB+)
} sd_card_type_t;

// SD card commands
#define CMD0            (0)              // GO_IDLE_STATE
#define CMD1            (1)              // SEND_OP_COND (MMC)
#define CMD8            (8)              // SEND_IF_COND
#define CMD9            (9)              // SEND_CSD
#define CMD10           (10)             // SEND_CID
#define CMD12           (12)             // STOP_TRANSMISSION
#define CMD16           (16)             // SET_BLOCKLEN
#define CMD17           (17)             // READ_SINGLE_BLOCK
#define CMD18           (18)             // READ_MULTIPLE_BLOCK
#define CMD23           (23)             // SET_BLOCK_COUNT
#define CMD24           (24)             // WRITE_BLOCK
#define CMD25           (25)             // WRITE_MULTIPLE_BLOCK
#define CMD55           (55)             // APP_CMD
#define CMD58           (58)             // READ_OCR
#define ACMD23          (23)             // SET_WR_BLK_ERASE_COUNT
#define ACMD41          (41)             // SD_SEND_OP_COND

// SD card response types
#define R1_IDLE_STATE           (1<<0)
#define R1_ERASE_RESET          (1<<1)
#define R1_ILLEGAL_COMMAND      (1<<2)
#define R1_COM_CRC_ERROR        (1<<3)
#define R1_ERASE_SEQUENCE_ERROR (1<<4)
#define R1_ADDRESS_ERROR        (1<<5)
#define R1_PARAMETER_ERROR      (1<<6)

// Data tokens
#define DATA_START_BLOCK        (0xFE)
#define DATA_START_BLOCK_MULT   (0xFC)
#define DATA_STOP_MULT          (0xFD)

// FAT32 constants
#define SECTOR_SIZE             (512)
#define MAX_FILENAME_LEN        (255)
#define MAX_PATH_LEN            (512)

// File system types
typedef enum {
    FS_TYPE_UNKNOWN = 0,
    FS_TYPE_FAT12,
    FS_TYPE_FAT16,
    FS_TYPE_FAT32
} fs_type_t;

// File attributes
#define ATTR_READ_ONLY          (0x01)
#define ATTR_HIDDEN             (0x02)
#define ATTR_SYSTEM             (0x04)
#define ATTR_VOLUME_ID          (0x08)
#define ATTR_DIRECTORY          (0x10)
#define ATTR_ARCHIVE            (0x20)
#define ATTR_LONG_NAME          (0x0F)

// Error codes
typedef enum {
    SD_OK = 0,
    SD_ERROR_NO_CARD,
    SD_ERROR_INIT_FAILED,
    SD_ERROR_READ_FAILED,
    SD_ERROR_WRITE_FAILED,
    SD_ERROR_NOT_MOUNTED,
    SD_ERROR_FILE_NOT_FOUND,
    SD_ERROR_INVALID_PATH,
    SD_ERROR_DISK_FULL,
    SD_ERROR_FILE_EXISTS,
    SD_ERROR_INVALID_PARAMETER
} sd_error_t;

// File handle structure
typedef struct {
    bool is_open;
    uint32_t start_cluster;
    uint32_t current_cluster;
    uint32_t file_size;
    uint32_t position;
    uint8_t attributes;
    char filename[13];  // 8.3 format + null terminator
} sd_file_t;

// Directory entry structure
typedef struct {
    char name[13];      // 8.3 format + null terminator
    uint8_t attributes;
    uint32_t size;
    uint16_t date;
    uint16_t time;
} sd_dir_entry_t;

// Boot sector structure (simplified)
typedef struct __attribute__((packed)) {
    uint8_t jump[3];
    char oem_name[8];
    uint16_t bytes_per_sector;
    uint8_t sectors_per_cluster;
    uint16_t reserved_sectors;
    uint8_t num_fats;
    uint16_t root_entries;
    uint16_t total_sectors_16;
    uint8_t media_type;
    uint16_t fat_size_16;
    uint16_t sectors_per_track;
    uint16_t num_heads;
    uint32_t hidden_sectors;
    uint32_t total_sectors_32;
    
    // FAT32 specific
    uint32_t fat_size_32;
    uint16_t ext_flags;
    uint16_t fs_version;
    uint32_t root_cluster;
    uint16_t fs_info;
    uint16_t backup_boot_sector;
    uint8_t reserved[12];
    uint8_t drive_number;
    uint8_t reserved1;
    uint8_t boot_signature;
    uint32_t volume_id;
    char volume_label[11];
    char fs_type[8];
} fat32_boot_sector_t;

// Function prototypes

// Low-level SD card functions
bool sd_card_present(void);
sd_error_t sd_init(void);
void sd_deinit(void);
bool sd_available(void);
void sd_acquire(void);
void sd_release(void);

// Block-level read/write functions
sd_error_t sd_read_block(uint32_t block, uint8_t *buffer);
sd_error_t sd_write_block(uint32_t block, const uint8_t *buffer);
sd_error_t sd_read_blocks(uint32_t start_block, uint32_t num_blocks, uint8_t *buffer);
sd_error_t sd_write_blocks(uint32_t start_block, uint32_t num_blocks, const uint8_t *buffer);

// File system functions
sd_error_t sd_mount(void);
void sd_unmount(void);
bool sd_is_mounted(void);
fs_type_t sd_get_fs_type(void);
sd_card_type_t sd_get_card_type(void);  // Get SD card type (SD/SDHC/SDXC)
uint32_t sd_get_free_space(void);
uint32_t sd_get_total_space(void);

// File operations
sd_error_t sd_file_open(sd_file_t *file, const char *filename);
sd_error_t sd_file_create(sd_file_t *file, const char *filename);
sd_error_t sd_file_close(sd_file_t *file);
sd_error_t sd_file_read(sd_file_t *file, void *buffer, size_t size, size_t *bytes_read);
sd_error_t sd_file_write(sd_file_t *file, const void *buffer, size_t size);
sd_error_t sd_file_seek(sd_file_t *file, uint32_t position);
uint32_t sd_file_tell(sd_file_t *file);
uint32_t sd_file_size(sd_file_t *file);
bool sd_file_eof(sd_file_t *file);
sd_error_t sd_file_delete(const char *filename);

// Directory operations
sd_error_t sd_dir_open(const char *path);
sd_error_t sd_dir_read(sd_dir_entry_t *entry);
sd_error_t sd_dir_close(void);
sd_error_t sd_dir_create(const char *dirname);
sd_error_t sd_dir_delete(const char *dirname);

// Utility functions
const char *sd_error_string(sd_error_t error);
sd_error_t sd_debug_init(void);
void sd_debug_dump_sector(uint32_t sector_num);
sd_error_t sd_list_root_directory(void);

#endif // SDCARD_H