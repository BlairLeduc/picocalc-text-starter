//
//  PicoCalc SD Card driver with FAT32 support
//
//  This driver provides block-level access to SD cards and implements
//  basic FAT32 file system operations for reading and writing files.
//
//  The driver uses SPI0 interface and follows the SD card SPI protocol.
//

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <strings.h>  // For strcasecmp

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico/sem.h"

#include "sdcard.h"

// Global state
static bool sd_initialised = false;
static bool sd_mounted = false;
static semaphore_t sd_sem;

// FAT32 file system state
static fat32_boot_sector_t boot_sector;
static fs_type_t fs_type = FS_TYPE_UNKNOWN;
static uint32_t partition_start_sector = 0;  // Offset for partitioned drives
static uint32_t fat_start_sector;
static uint32_t cluster_start_sector;
static uint32_t root_dir_first_cluster;
static uint32_t sectors_per_cluster;
static uint32_t bytes_per_cluster;

// Directory state
static bool dir_open = false;
static uint32_t dir_current_cluster;
static uint32_t dir_current_sector;
static uint16_t dir_current_entry;

// Working buffers
static uint8_t sector_buffer[SECTOR_SIZE] __attribute__((aligned(4)));

//
// Low-level SD card SPI functions
//

static inline void sd_cs_select(void) {
    gpio_put(SD_CS, 0);
}

static inline void sd_cs_deselect(void) {
    gpio_put(SD_CS, 1);
}

static uint8_t sd_spi_write_read(uint8_t data) {
    uint8_t result;
    spi_write_read_blocking(SD_SPI, &data, &result, 1);
    return result;
}

static void sd_spi_write_buf(const uint8_t *src, size_t len) {
    spi_write_blocking(SD_SPI, src, len);
}

static void sd_spi_read_buf(uint8_t *dst, size_t len) {
    // Send dummy bytes while reading
    memset(dst, 0xFF, len);
    spi_write_read_blocking(SD_SPI, dst, dst, len);
}

static bool sd_wait_ready(void) {
    uint8_t response;
    uint32_t timeout = 10000;  // Add timeout to prevent infinite loop
    do {
        response = sd_spi_write_read(0xFF);
        timeout--;
        if (timeout == 0) {
            return false;  // Timeout occurred
        }
    } while (response != 0xFF);
    return true;  // Success
}

static uint8_t sd_send_command(uint8_t cmd, uint32_t arg) {
    uint8_t response;
    uint8_t retry = 0;
    
    // For CMD0, don't wait for ready state since card might not be ready yet
    if (cmd != CMD0) {
        if (!sd_wait_ready()) {
            return 0xFF;  // Timeout waiting for ready
        }
    }
    
    // Prepare command packet
    uint8_t packet[6];
    packet[0] = 0x40 | cmd;
    packet[1] = (arg >> 24) & 0xFF;
    packet[2] = (arg >> 16) & 0xFF;
    packet[3] = (arg >> 8) & 0xFF;
    packet[4] = arg & 0xFF;
    
    // Calculate CRC for specific commands
    uint8_t crc = 0xFF;
    if (cmd == CMD0) crc = 0x95;
    if (cmd == CMD8) crc = 0x87;
    packet[5] = crc;
    
    // Send command
    sd_cs_select();
    sd_spi_write_buf(packet, 6);
    
    // Wait for response (R1) - but with timeout
    response = 0xFF;
    do {
        response = sd_spi_write_read(0xFF);
        retry++;
    } while ((response & 0x80) && (retry < 64));  // Increased timeout from 10 to 64
    
    // Don't deselect here - let caller handle it
    return response;
}

//
// Card detection and initialization
//

bool sd_card_present(void) {
    return !gpio_get(SD_DETECT);  // Active low
}

sd_error_t sd_init(void) {
    if (sd_initialised) {
        return SD_OK;
    }
    
    if (!sd_card_present()) {
        return SD_ERROR_NO_CARD;
    }
    
    // Initialize GPIO
    gpio_init(SD_MISO);
    gpio_init(SD_CS);
    gpio_init(SD_SCK);
    gpio_init(SD_MOSI);
    gpio_init(SD_DETECT);
    
    gpio_set_dir(SD_CS, GPIO_OUT);
    gpio_set_dir(SD_DETECT, GPIO_IN);
    gpio_pull_up(SD_DETECT);
    
    // Start with lower SPI speed for initialization (400kHz)
    spi_init(SD_SPI, 400000);
    gpio_set_function(SD_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SD_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SD_MOSI, GPIO_FUNC_SPI);
    
    // Initialize semaphore
    sem_init(&sd_sem, 1, 1);
    
    // Ensure CS is high and wait for card to stabilize
    sd_cs_deselect();
    sleep_ms(10);  // Give card time to power up
    
    // Send 80+ clock pulses with CS high to put card in SPI mode
    for (int i = 0; i < 20; i++) {  // Increased from 10 to 20
        sd_spi_write_read(0xFF);
    }
    
    sleep_ms(1);  // Additional delay before first command
    
    // Reset card to SPI mode (CMD0) - try multiple times
    uint8_t response;
    int cmd0_attempts = 0;
    do {
        response = sd_send_command(CMD0, 0);
        sd_cs_deselect();
        cmd0_attempts++;
        if (response != R1_IDLE_STATE && cmd0_attempts < 10) {
            sleep_ms(10);  // Wait before retry
        }
    } while (response != R1_IDLE_STATE && cmd0_attempts < 10);
    
    if (response != R1_IDLE_STATE) {
        return SD_ERROR_INIT_FAILED;
    }
    
    // Check interface condition (CMD8)
    response = sd_send_command(CMD8, 0x1AA);
    if (response == R1_IDLE_STATE) {
        // Read the rest of R7 response
        uint8_t r7[4];
        sd_spi_read_buf(r7, 4);
        sd_cs_deselect();
        
        // Check if voltage range is acceptable
        if ((r7[2] & 0x0F) != 0x01 || r7[3] != 0xAA) {
            return SD_ERROR_INIT_FAILED;
        }
    } else {
        sd_cs_deselect();
    }
    
    // Initialize card with ACMD41
    uint32_t timeout = 1000;
    do {
        // Send CMD55 (APP_CMD) followed by ACMD41
        response = sd_send_command(CMD55, 0);
        sd_cs_deselect();
        
        if (response > 1) {
            return SD_ERROR_INIT_FAILED;
        }
        
        response = sd_send_command(ACMD41, 0x40000000);  // HCS bit for SDHC support
        sd_cs_deselect();
        
        if (response == 0) break;
        
        sleep_ms(1);
        timeout--;
    } while (timeout > 0);
    
    if (timeout == 0) {
        return SD_ERROR_INIT_FAILED;
    }
    
    // Set block length to 512 bytes
    response = sd_send_command(CMD16, SECTOR_SIZE);
    sd_cs_deselect();
    
    if (response != 0) {
        return SD_ERROR_INIT_FAILED;
    }
    
    // Switch to higher speed for normal operation
    spi_set_baudrate(SD_SPI, SD_BAUDRATE);
    
    sd_initialised = true;
    return SD_OK;
}

void sd_deinit(void) {
    if (sd_mounted) {
        sd_unmount();
    }
    sd_initialised = false;
}

bool sd_available(void) {
    return sem_available(&sd_sem);
}

void sd_acquire(void) {
    sem_acquire_blocking(&sd_sem);
}

void sd_release(void) {
    sem_release(&sd_sem);
}

//
// Block-level read/write operations
//

sd_error_t sd_read_block(uint32_t block, uint8_t *buffer) {
    if (!sd_initialised) return SD_ERROR_INIT_FAILED;
    if (!sd_card_present()) return SD_ERROR_NO_CARD;
    
    uint8_t response = sd_send_command(CMD17, block);
    if (response != 0) {
        sd_cs_deselect();
        return SD_ERROR_READ_FAILED;
    }
    
    // Wait for data token
    uint32_t timeout = 100000;
    do {
        response = sd_spi_write_read(0xFF);
        timeout--;
    } while (response != DATA_START_BLOCK && timeout > 0);
    
    if (timeout == 0) {
        sd_cs_deselect();
        return SD_ERROR_READ_FAILED;
    }
    
    // Read data
    sd_spi_read_buf(buffer, SECTOR_SIZE);
    
    // Read CRC (ignore it)
    sd_spi_write_read(0xFF);
    sd_spi_write_read(0xFF);
    
    sd_cs_deselect();
    return SD_OK;
}

sd_error_t sd_write_block(uint32_t block, const uint8_t *buffer) {
    if (!sd_initialised) return SD_ERROR_INIT_FAILED;
    if (!sd_card_present()) return SD_ERROR_NO_CARD;
    
    uint8_t response = sd_send_command(CMD24, block);
    if (response != 0) {
        sd_cs_deselect();
        return SD_ERROR_WRITE_FAILED;
    }
    
    // Send data token
    sd_spi_write_read(DATA_START_BLOCK);
    
    // Send data
    sd_spi_write_buf(buffer, SECTOR_SIZE);
    
    // Send dummy CRC
    sd_spi_write_read(0xFF);
    sd_spi_write_read(0xFF);
    
    // Check data response
    response = sd_spi_write_read(0xFF) & 0x1F;
    sd_cs_deselect();
    
    if (response != 0x05) {
        return SD_ERROR_WRITE_FAILED;
    }
    
    // Wait for programming to finish
    sd_cs_select();
    sd_wait_ready();
    sd_cs_deselect();
    
    return SD_OK;
}

sd_error_t sd_read_blocks(uint32_t start_block, uint32_t num_blocks, uint8_t *buffer) {
    for (uint32_t i = 0; i < num_blocks; i++) {
        sd_error_t result = sd_read_block(start_block + i, buffer + (i * SECTOR_SIZE));
        if (result != SD_OK) {
            return result;
        }
    }
    return SD_OK;
}

sd_error_t sd_write_blocks(uint32_t start_block, uint32_t num_blocks, const uint8_t *buffer) {
    for (uint32_t i = 0; i < num_blocks; i++) {
        sd_error_t result = sd_write_block(start_block + i, buffer + (i * SECTOR_SIZE));
        if (result != SD_OK) {
            return result;
        }
    }
    return SD_OK;
}

//
// FAT32 file system functions
//

static uint32_t cluster_to_sector(uint32_t cluster) {
    return partition_start_sector + cluster_start_sector + ((cluster - 2) * sectors_per_cluster);
}

static bool is_valid_fat32_boot_sector(const fat32_boot_sector_t *bs) {
    // Check bytes per sector - this is critical
    if (bs->bytes_per_sector != SECTOR_SIZE) {
        printf("Invalid bytes per sector: %d (expected %d)\n", bs->bytes_per_sector, SECTOR_SIZE);
        return false;
    }
    
    // Check sectors per cluster (must be power of 2)
    uint8_t spc = bs->sectors_per_cluster;
    if (spc == 0 || (spc & (spc - 1)) != 0) {
        printf("Invalid sectors per cluster: %d\n", spc);
        return false;
    }
    
    // Check number of FATs
    if (bs->num_fats == 0 || bs->num_fats > 2) {
        printf("Invalid number of FATs: %d\n", bs->num_fats);
        return false;
    }
    
    // Check if we have valid reserved sectors
    if (bs->reserved_sectors == 0) {
        printf("Invalid reserved sectors: %d\n", bs->reserved_sectors);
        return false;
    }
    
    return true;
}

sd_error_t sd_mount(void) {
    if (!sd_initialised) {
        sd_error_t result = sd_init();
        if (result != SD_OK) return result;
    }
    
    if (sd_mounted) return SD_OK;
    
    sd_acquire();
    
    // Read boot sector
    sd_error_t result = sd_read_block(0, sector_buffer);
    if (result != SD_OK) {
        printf("SD Mount: Failed to read boot sector, error: %s\n", sd_error_string(result));
        sd_release();
        return result;
    }
    
    // Check if this looks like a partition table (MBR) instead of a boot sector
    if (sector_buffer[510] == 0x55 && sector_buffer[511] == 0xAA && 
        sector_buffer[0] != 0xEB && sector_buffer[0] != 0xE9) {
        
        // Read partition table entry (offset 446)
        uint8_t *partition_entry = sector_buffer + 446;
        if (partition_entry[4] == 0x0B || partition_entry[4] == 0x0C || partition_entry[4] == 0x06) {
            // FAT32 or FAT16 partition
            partition_start_sector = (partition_entry[11] << 24) | (partition_entry[10] << 16) |
                                   (partition_entry[9] << 8) | partition_entry[8];
            
            // Read the actual boot sector from the partition
            result = sd_read_block(partition_start_sector, sector_buffer);
            if (result != SD_OK) {
                printf("SD Mount: Failed to read partition boot sector\n");
                sd_release();
                return result;
            }
        } else {
            printf("SD Mount: No supported partition found\n");
            sd_release();
            return SD_ERROR_NOT_MOUNTED;
        }
    } else {
        // No partition table, filesystem starts at sector 0
        partition_start_sector = 0;
        printf("SD Mount: No partition table detected, filesystem at sector 0\n");
    }
    
    // Copy boot sector data
    memcpy(&boot_sector, sector_buffer, sizeof(fat32_boot_sector_t));
    
    // Print some boot sector info
    printf("OEM Name: %.8s\n", boot_sector.oem_name);
    printf("Bytes per sector: %d\n", boot_sector.bytes_per_sector);
    printf("Sectors per cluster: %d\n", boot_sector.sectors_per_cluster);
    printf("Reserved sectors: %d\n", boot_sector.reserved_sectors);
    printf("Number of FATs: %d\n", boot_sector.num_fats);
    
    // Validate boot sector
    if (!is_valid_fat32_boot_sector(&boot_sector)) {
        printf("SD Mount: Boot sector validation failed\n");
        sd_release();
        return SD_ERROR_NOT_MOUNTED;
    }
    
    // Determine file system type
    uint32_t total_sectors = boot_sector.total_sectors_16 ? 
                           boot_sector.total_sectors_16 : boot_sector.total_sectors_32;
    uint32_t fat_size = boot_sector.fat_size_16 ? 
                       boot_sector.fat_size_16 : boot_sector.fat_size_32;
    uint32_t root_dir_sectors = ((boot_sector.root_entries * 32) + 
                                (boot_sector.bytes_per_sector - 1)) / boot_sector.bytes_per_sector;
    uint32_t data_sectors = total_sectors - (boot_sector.reserved_sectors + 
                           (boot_sector.num_fats * fat_size) + root_dir_sectors);
    uint32_t count_of_clusters = data_sectors / boot_sector.sectors_per_cluster;
    
    if (count_of_clusters < 4085) {
        fs_type = FS_TYPE_FAT12;
        printf("Detected FAT12 filesystem (%lu clusters)\n", count_of_clusters);
    } else if (count_of_clusters < 65525) {
        fs_type = FS_TYPE_FAT16;
        printf("Detected FAT16 filesystem (%lu clusters)\n", count_of_clusters);
    } else {
        fs_type = FS_TYPE_FAT32;
        printf("Detected FAT32 filesystem (%lu clusters)\n", count_of_clusters);
    }
    
    // Calculate important sectors
    fat_start_sector = boot_sector.reserved_sectors;
    cluster_start_sector = boot_sector.reserved_sectors + 
                          (boot_sector.num_fats * fat_size) + root_dir_sectors;
    
    if (fs_type == FS_TYPE_FAT32) {
        root_dir_first_cluster = boot_sector.root_cluster;
    } else {
        root_dir_first_cluster = 0;  // Root directory is in a fixed location for FAT16/12
    }
    
    sectors_per_cluster = boot_sector.sectors_per_cluster;
    bytes_per_cluster = sectors_per_cluster * SECTOR_SIZE;
    
    sd_mounted = true;
    sd_release();
    return SD_OK;
}

void sd_unmount(void) {
    if (dir_open) {
        sd_dir_close();
    }
    sd_mounted = false;
    fs_type = FS_TYPE_UNKNOWN;
    partition_start_sector = 0;
}

bool sd_is_mounted(void) {
    return sd_mounted;
}

fs_type_t sd_get_fs_type(void) {
    return fs_type;
}

uint32_t sd_get_free_space(void) {
    // This is a simplified implementation
    // A full implementation would read the FAT to count free clusters
    return 0;
}

uint32_t sd_get_total_space(void) {
    if (!sd_mounted) return 0;
    
    uint32_t total_sectors = boot_sector.total_sectors_16 ? 
                           boot_sector.total_sectors_16 : boot_sector.total_sectors_32;
    return total_sectors * SECTOR_SIZE;
}

//
// File system utility functions
//

static void convert_filename_to_83(const char *filename, char *name83) {
    memset(name83, ' ', 11);
    name83[11] = '\0';
    
    const char *dot = strrchr(filename, '.');
    int name_len = dot ? (dot - filename) : strlen(filename);
    
    // Copy name part (max 8 characters)
    for (int i = 0; i < name_len && i < 8; i++) {
        name83[i] = toupper(filename[i]);
    }
    
    // Copy extension part (max 3 characters)
    if (dot && strlen(dot + 1) > 0) {
        for (int i = 0; i < 3 && dot[1 + i]; i++) {
            name83[8 + i] = toupper(dot[1 + i]);
        }
    }
}

static void convert_83_to_filename(const char *name83, char *filename) {
    int pos = 0;
    
    // Copy name part
    for (int i = 0; i < 8 && name83[i] != ' '; i++) {
        filename[pos++] = tolower(name83[i]);
    }
    
    // Copy extension part
    bool has_ext = false;
    for (int i = 8; i < 11; i++) {
        if (name83[i] != ' ') {
            if (!has_ext) {
                filename[pos++] = '.';
                has_ext = true;
            }
            filename[pos++] = tolower(name83[i]);
        }
    }
    
    filename[pos] = '\0';
}

// Long filename support
#define LFN_ATTR 0x0F
#define LFN_LAST_ENTRY 0x40

typedef struct {
    uint8_t seq;           // Sequence number
    uint16_t name1[5];     // First 5 characters (UTF-16)
    uint8_t attr;          // Always 0x0F for LFN
    uint8_t type;          // Always 0 for LFN
    uint8_t checksum;      // Checksum of 8.3 name
    uint16_t name2[6];     // Next 6 characters (UTF-16)
    uint16_t first_clus;   // Always 0 for LFN
    uint16_t name3[2];     // Last 2 characters (UTF-16)
} __attribute__((packed)) lfn_entry_t;

// Calculate checksum for 8.3 filename
static uint8_t lfn_checksum(const uint8_t *name83) {
    uint8_t sum = 0;
    for (int i = 0; i < 11; i++) {
        sum = ((sum & 1) << 7) + (sum >> 1) + name83[i];
    }
    return sum;
}

// Convert UTF-16 to UTF-8 (simplified - only handles ASCII range)
static void utf16_to_utf8(const uint16_t *utf16, char *utf8, int max_len) {
    int pos = 0;
    for (int i = 0; i < max_len - 1 && utf16[i] != 0 && utf16[i] != 0xFFFF; i++) {
        if (utf16[i] < 0x80) {  // ASCII range
            utf8[pos++] = (char)utf16[i];
        } else {
            utf8[pos++] = '?';  // Non-ASCII placeholder
        }
    }
    utf8[pos] = '\0';
}

// Extract long filename from LFN entries
static bool extract_long_filename(uint8_t *dir_entries, int start_idx, int end_idx, char *filename, int max_len) {
    char lfn_parts[64] = {0};  // Temporary buffer for LFN parts
    int total_chars = 0;
    uint8_t expected_checksum = 0;
    bool first_entry = true;
    
    // Process LFN entries in reverse order (they're stored backwards)
    for (int i = start_idx; i <= end_idx; i++) {
        lfn_entry_t *lfn = (lfn_entry_t *)(dir_entries + (i * 32));
        
        if (lfn->attr != LFN_ATTR) {
            return false;  // Not an LFN entry
        }
        
        if (first_entry) {
            expected_checksum = lfn->checksum;
            first_entry = false;
        } else if (lfn->checksum != expected_checksum) {
            return false;  // Checksum mismatch
        }
        
        // Extract characters from this LFN entry
        char part[14];  // 13 chars + null terminator
        int part_pos = 0;
        
        // Extract from name1 (5 chars)
        for (int j = 0; j < 5 && part_pos < 13; j++) {
            if (lfn->name1[j] != 0 && lfn->name1[j] != 0xFFFF) {
                part[part_pos++] = (lfn->name1[j] < 0x80) ? (char)lfn->name1[j] : '?';
            }
        }
        
        // Extract from name2 (6 chars)
        for (int j = 0; j < 6 && part_pos < 13; j++) {
            if (lfn->name2[j] != 0 && lfn->name2[j] != 0xFFFF) {
                part[part_pos++] = (lfn->name2[j] < 0x80) ? (char)lfn->name2[j] : '?';
            }
        }
        
        // Extract from name3 (2 chars)
        for (int j = 0; j < 2 && part_pos < 13; j++) {
            if (lfn->name3[j] != 0 && lfn->name3[j] != 0xFFFF) {
                part[part_pos++] = (lfn->name3[j] < 0x80) ? (char)lfn->name3[j] : '?';
            }
        }
        
        part[part_pos] = '\0';
        
        // Prepend this part to the filename (LFN entries are in reverse order)
        int part_len = strlen(part);
        if (total_chars + part_len < max_len - 1) {
            memmove(lfn_parts + part_len, lfn_parts, total_chars + 1);
            memcpy(lfn_parts, part, part_len);
            total_chars += part_len;
        }
    }
    
    // Verify checksum against 8.3 entry
    uint8_t *dot_entry = dir_entries + ((end_idx + 1) * 32);
    uint8_t actual_checksum = lfn_checksum(dot_entry);
    
    if (actual_checksum != expected_checksum) {
        return false;  // Checksum verification failed
    }
    
    strncpy(filename, lfn_parts, max_len - 1);
    filename[max_len - 1] = '\0';
    return true;
}

//
// File operations (simplified implementation)
//

sd_error_t sd_file_open(sd_file_t *file, const char *filename) {
    if (!sd_mounted) return SD_ERROR_NOT_MOUNTED;
    if (!file || !filename) return SD_ERROR_INVALID_PARAMETER;
    
    // This is a simplified implementation that only looks in the root directory
    // A full implementation would support subdirectories and long filenames
    
    memset(file, 0, sizeof(sd_file_t));
    
    char name83[12];
    convert_filename_to_83(filename, name83);
    
    sd_acquire();
    
    // Calculate root directory location - same logic as in sd_list_root_directory
    uint32_t root_sector;
    uint32_t max_entries;
    
    if (fs_type == FS_TYPE_FAT32) {
        // FAT32: root directory is in clusters
        root_sector = cluster_to_sector(root_dir_first_cluster);
        max_entries = bytes_per_cluster / 32;  // 32 bytes per directory entry
    } else {
        // FAT16/12: root directory is in a fixed location
        uint32_t fat_size = boot_sector.fat_size_16;
        uint32_t root_sector_relative = boot_sector.reserved_sectors + (boot_sector.num_fats * fat_size);
        root_sector = partition_start_sector + root_sector_relative;
        max_entries = boot_sector.root_entries;
    }
    
    uint32_t sectors_to_read = (max_entries * 32 + SECTOR_SIZE - 1) / SECTOR_SIZE;
    
    // Search through all directory sectors
    for (uint32_t sector_offset = 0; sector_offset < sectors_to_read && sector_offset < 8; sector_offset++) {
        uint32_t current_sector = root_sector + sector_offset;
        
        sd_error_t result = sd_read_block(current_sector, sector_buffer);
        if (result != SD_OK) {
            sd_release();
            return result;
        }
        
        // Look for the file in directory entries
        for (int i = 0; i < SECTOR_SIZE / 32; i++) {
            uint8_t *entry = sector_buffer + (i * 32);
            
            if (entry[0] == 0x00) {
                // End of directory - file not found
                sd_release();
                return SD_ERROR_FILE_NOT_FOUND;
            }
            if (entry[0] == 0xE5) continue;  // Deleted entry
            
            // Check if this is an LFN entry sequence
            if (entry[11] == LFN_ATTR) {
                // Look for complete LFN sequence
                int lfn_start = i;
                int lfn_count = 0;
                int scan_idx = i;
                
                // Count consecutive LFN entries
                while (scan_idx < SECTOR_SIZE / 32) {
                    uint8_t *scan_entry = sector_buffer + (scan_idx * 32);
                    if (scan_entry[0] == 0x00) break;
                    if (scan_entry[0] == 0xE5) break;
                    if (scan_entry[11] != LFN_ATTR) break;
                    lfn_count++;
                    scan_idx++;
                }
                
                // Check for valid 8.3 entry after LFN entries
                if (scan_idx < SECTOR_SIZE / 32) {
                    uint8_t *dot_entry = sector_buffer + (scan_idx * 32);
                    if (dot_entry[0] != 0x00 && dot_entry[0] != 0xE5 && 
                        (dot_entry[11] & 0x0F) != LFN_ATTR &&
                        !(dot_entry[11] & 0x08) && !(dot_entry[11] & 0x10)) {  // Not volume label or directory
                        
                        // Extract long filename and check for match
                        char long_filename[256];
                        if (extract_long_filename(sector_buffer, lfn_start, lfn_start + lfn_count - 1, 
                                                 long_filename, sizeof(long_filename))) {
                            
                            // Check if this matches our target filename (case-insensitive)
                            bool matches = (strcasecmp(filename, long_filename) == 0);
                            
                            // Also check 8.3 name for compatibility
                            if (!matches && memcmp(dot_entry, name83, 11) == 0) {
                                matches = true;
                            }
                            
                            if (matches) {
                                // Found the file
                                file->is_open = true;
                                file->start_cluster = (dot_entry[21] << 8) | dot_entry[20];  // High cluster
                                file->start_cluster = (file->start_cluster << 16) | (dot_entry[27] << 8) | dot_entry[26];  // Low cluster
                                file->current_cluster = file->start_cluster;
                                file->file_size = (dot_entry[31] << 24) | (dot_entry[30] << 16) | (dot_entry[29] << 8) | dot_entry[28];
                                file->position = 0;
                                file->attributes = dot_entry[11];
                                strncpy(file->filename, filename, 12);
                                
                                sd_release();
                                return SD_OK;
                            }
                        }
                        
                        // Skip past the LFN sequence and 8.3 entry
                        i = scan_idx;
                        continue;
                    }
                }
                
                // If LFN processing failed, skip this entry
                continue;
            }
            
            // Standard 8.3 entry processing
            if (entry[11] & ATTR_VOLUME_ID) continue;  // Volume label
            if (entry[11] & ATTR_DIRECTORY) continue;  // Directory
            if (entry[11] & 0x0F) continue;  // Remaining long filename entries
            
            if (memcmp(entry, name83, 11) == 0) {
                // Found the file
                file->is_open = true;
                file->start_cluster = (entry[21] << 8) | entry[20];  // High cluster
                file->start_cluster = (file->start_cluster << 16) | (entry[27] << 8) | entry[26];  // Low cluster
                file->current_cluster = file->start_cluster;
                file->file_size = (entry[31] << 24) | (entry[30] << 16) | (entry[29] << 8) | entry[28];
                file->position = 0;
                file->attributes = entry[11];
                strncpy(file->filename, filename, 12);
                
                sd_release();
                return SD_OK;
            }
        }
    }
    
    sd_release();
    return SD_ERROR_FILE_NOT_FOUND;
}

sd_error_t sd_file_create(sd_file_t *file, const char *filename) {
    // This is a placeholder implementation
    // Creating files requires finding free directory entries and clusters
    return SD_ERROR_INVALID_PARAMETER;
}

sd_error_t sd_file_close(sd_file_t *file) {
    if (!file || !file->is_open) {
        return SD_ERROR_INVALID_PARAMETER;
    }
    
    memset(file, 0, sizeof(sd_file_t));
    return SD_OK;
}

sd_error_t sd_file_read(sd_file_t *file, void *buffer, size_t size, size_t *bytes_read) {
    if (!file || !file->is_open || !buffer) {
        return SD_ERROR_INVALID_PARAMETER;
    }
    
    if (bytes_read) *bytes_read = 0;
    
    if (file->position >= file->file_size) {
        return SD_OK;  // EOF
    }
    
    size_t remaining = file->file_size - file->position;
    if (size > remaining) size = remaining;
    
    sd_acquire();
    
    size_t total_read = 0;
    uint8_t *dest = (uint8_t *)buffer;
    
    while (total_read < size) {
        uint32_t cluster_offset = file->position % bytes_per_cluster;
        uint32_t sector_in_cluster = cluster_offset / SECTOR_SIZE;
        uint32_t byte_in_sector = cluster_offset % SECTOR_SIZE;
        
        uint32_t sector = cluster_to_sector(file->current_cluster) + sector_in_cluster;
        
        sd_error_t result = sd_read_block(sector, sector_buffer);
        if (result != SD_OK) {
            sd_release();
            return result;
        }
        
        size_t bytes_to_copy = SECTOR_SIZE - byte_in_sector;
        if (bytes_to_copy > size - total_read) {
            bytes_to_copy = size - total_read;
        }
        
        memcpy(dest + total_read, sector_buffer + byte_in_sector, bytes_to_copy);
        total_read += bytes_to_copy;
        file->position += bytes_to_copy;
        
        // Check if we need to move to the next cluster
        if ((file->position % bytes_per_cluster) == 0 && total_read < size) {
            // This is simplified - a full implementation would read the FAT
            // to find the next cluster in the chain
            file->current_cluster++;
        }
    }
    
    if (bytes_read) *bytes_read = total_read;
    
    sd_release();
    return SD_OK;
}

sd_error_t sd_file_write(sd_file_t *file, const void *buffer, size_t size) {
    // This is a placeholder implementation
    // Writing files requires cluster allocation and FAT updates
    return SD_ERROR_INVALID_PARAMETER;
}

sd_error_t sd_file_seek(sd_file_t *file, uint32_t position) {
    if (!file || !file->is_open) {
        return SD_ERROR_INVALID_PARAMETER;
    }
    
    if (position > file->file_size) {
        position = file->file_size;
    }
    
    file->position = position;
    
    // Recalculate current cluster (simplified)
    uint32_t cluster_num = position / bytes_per_cluster;
    file->current_cluster = file->start_cluster + cluster_num;
    
    return SD_OK;
}

uint32_t sd_file_tell(sd_file_t *file) {
    return file ? file->position : 0;
}

uint32_t sd_file_size(sd_file_t *file) {
    return file ? file->file_size : 0;
}

bool sd_file_eof(sd_file_t *file) {
    return file ? (file->position >= file->file_size) : true;
}

sd_error_t sd_file_delete(const char *filename) {
    // This is a placeholder implementation
    return SD_ERROR_INVALID_PARAMETER;
}

//
// Directory operations (placeholder implementations)
//

sd_error_t sd_dir_open(const char *path) {
    return SD_ERROR_INVALID_PARAMETER;
}

sd_error_t sd_dir_read(sd_dir_entry_t *entry) {
    return SD_ERROR_INVALID_PARAMETER;
}

sd_error_t sd_dir_close(void) {
    dir_open = false;
    return SD_OK;
}

sd_error_t sd_dir_create(const char *dirname) {
    return SD_ERROR_INVALID_PARAMETER;
}

sd_error_t sd_dir_delete(const char *dirname) {
    return SD_ERROR_INVALID_PARAMETER;
}

//
// Utility functions
//

const char *sd_error_string(sd_error_t error) {
    switch (error) {
        case SD_OK: return "Success";
        case SD_ERROR_NO_CARD: return "No SD card present";
        case SD_ERROR_INIT_FAILED: return "SD card initialization failed";
        case SD_ERROR_READ_FAILED: return "Read operation failed";
        case SD_ERROR_WRITE_FAILED: return "Write operation failed";
        case SD_ERROR_NOT_MOUNTED: return "File system not mounted";
        case SD_ERROR_FILE_NOT_FOUND: return "File not found";
        case SD_ERROR_INVALID_PATH: return "Invalid path";
        case SD_ERROR_DISK_FULL: return "Disk full";
        case SD_ERROR_FILE_EXISTS: return "File already exists";
        case SD_ERROR_INVALID_PARAMETER: return "Invalid parameter";
        default: return "Unknown error";
    }
}

// Debug function to test basic SPI communication
sd_error_t sd_debug_init(void) {
    printf("SD Debug Init: Starting...\n");
    
    if (!sd_card_present()) {
        printf("SD Debug: No card detected\n");
        return SD_ERROR_NO_CARD;
    }
    printf("SD Debug: Card detected\n");
    
    // Initialize GPIO
    gpio_init(SD_MISO);
    gpio_init(SD_CS);
    gpio_init(SD_SCK);
    gpio_init(SD_MOSI);
    gpio_init(SD_DETECT);
    
    gpio_set_dir(SD_CS, GPIO_OUT);
    gpio_set_dir(SD_DETECT, GPIO_IN);
    gpio_pull_up(SD_DETECT);
    
    // Start with very low speed
    printf("SD Debug: Initializing SPI at 100kHz\n");
    spi_init(SD_SPI, 100000);  // Very slow for debugging
    gpio_set_function(SD_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SD_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SD_MOSI, GPIO_FUNC_SPI);
    
    // Initialize semaphore
    sem_init(&sd_sem, 1, 1);
    
    printf("SD Debug: Sending clock pulses\n");
    sd_cs_deselect();
    sleep_ms(50);  // Longer delay
    
    // Send many clock pulses
    for (int i = 0; i < 100; i++) {
        sd_spi_write_read(0xFF);
    }
    
    printf("SD Debug: Attempting CMD0\n");
    
    // Try CMD0 multiple times with debug output
    for (int attempt = 0; attempt < 10; attempt++) {
        printf("SD Debug: CMD0 attempt %d\n", attempt + 1);
        
        sd_cs_select();
        
        // Send CMD0 manually with debug
        uint8_t cmd0[] = {0x40, 0x00, 0x00, 0x00, 0x00, 0x95};
        for (int i = 0; i < 6; i++) {
            sd_spi_write_read(cmd0[i]);
        }
        
        // Wait for response
        uint8_t response = 0xFF;
        for (int i = 0; i < 64; i++) {
            response = sd_spi_write_read(0xFF);
            if ((response & 0x80) == 0) break;  // Valid response
        }
        
        sd_cs_deselect();
        
        printf("SD Debug: CMD0 response: 0x%02X\n", response);
        
        if (response == 0x01) {  // R1_IDLE_STATE
            printf("SD Debug: CMD0 success!\n");
            return SD_OK;
        }
        
        sleep_ms(100);
    }
    
    printf("SD Debug: CMD0 failed after all attempts\n");
    return SD_ERROR_INIT_FAILED;
}

// Debug function to dump sector contents
void sd_debug_dump_sector(uint32_t sector_num) {
    if (!sd_initialised) {
        printf("SD not initialized\n");
        return;
    }
    
    sd_acquire();
    
    sd_error_t result = sd_read_block(sector_num, sector_buffer);
    if (result != SD_OK) {
        printf("Failed to read sector %lu: %s\n", sector_num, sd_error_string(result));
        sd_release();
        return;
    }
    
    printf("Sector %lu contents:\n", sector_num);
    for (int i = 0; i < SECTOR_SIZE; i += 16) {
        printf("%04X: ", i);
        for (int j = 0; j < 16 && (i + j) < SECTOR_SIZE; j++) {
            printf("%02X ", sector_buffer[i + j]);
        }
        printf(" ");
        for (int j = 0; j < 16 && (i + j) < SECTOR_SIZE; j++) {
            char c = sector_buffer[i + j];
            printf("%c", (c >= 32 && c <= 126) ? c : '.');
        }
        printf("\n");
        
        // Only print first few lines and last few lines to avoid overwhelming output
        if (i >= 64 && i < 496) {
            if (i == 80) {
                printf("... (middle truncated) ...\n");
            }
            continue;
        }
    }
    
    sd_release();
}

// Simple directory listing function
sd_error_t sd_list_root_directory(void) {
    if (!sd_mounted) return SD_ERROR_NOT_MOUNTED;
    
    sd_acquire();
    
    printf("Files in root directory:\n");
    
    // Calculate root directory location
    uint32_t root_sector;
    uint32_t max_entries;
    
    if (fs_type == FS_TYPE_FAT32) {
        // FAT32: root directory is in clusters
        root_sector = cluster_to_sector(root_dir_first_cluster);
        max_entries = bytes_per_cluster / 32;  // 32 bytes per directory entry
    } else {
        // FAT16/12: root directory is in a fixed location
        uint32_t fat_size = boot_sector.fat_size_16;
        uint32_t root_sector_relative = boot_sector.reserved_sectors + (boot_sector.num_fats * fat_size);
        root_sector = partition_start_sector + root_sector_relative;
        max_entries = boot_sector.root_entries;
    }
    
    printf("%-28s %10s\n", "Name", "Size");
    printf("---------------------------------------\n");
    
    uint32_t entries_found = 0;
    uint32_t sectors_to_read = (max_entries * 32 + SECTOR_SIZE - 1) / SECTOR_SIZE;
    
    for (uint32_t sector_offset = 0; sector_offset < sectors_to_read && sector_offset < 8; sector_offset++) {
        uint32_t current_sector = root_sector + sector_offset;
        
        sd_error_t result = sd_read_block(current_sector, sector_buffer);
        if (result != SD_OK) {
            printf("Error reading directory sector %lu: %s\n", current_sector, sd_error_string(result));
            sd_release();
            return result;
        }
        
        // Process directory entries in this sector
        for (int entry_idx = 0; entry_idx < SECTOR_SIZE / 32; entry_idx++) {
            uint8_t *entry = sector_buffer + (entry_idx * 32);
            
            // Check for end of directory
            if (entry[0] == 0x00) {
                printf("\nTotal: %lu files found\n", entries_found);
                sd_release();
                return SD_OK;
            }
            
            // Skip deleted entries
            if (entry[0] == 0xE5) {
                continue;
            }
            
            // Check if this is an LFN entry
            if (entry[11] == LFN_ATTR) {
                // Look ahead for the complete LFN sequence
                int lfn_start = entry_idx;
                int lfn_count = 0;
                int scan_idx = entry_idx;
                
                // Count consecutive LFN entries
                while (scan_idx < SECTOR_SIZE / 32) {
                    uint8_t *scan_entry = sector_buffer + (scan_idx * 32);
                    if (scan_entry[0] == 0x00) break;
                    if (scan_entry[0] == 0xE5) break;
                    if (scan_entry[11] != LFN_ATTR) break;
                    lfn_count++;
                    scan_idx++;
                }
                
                // Check if there's a valid 8.3 entry after the LFN entries
                if (scan_idx < SECTOR_SIZE / 32) {
                    uint8_t *dot_entry = sector_buffer + (scan_idx * 32);
                    if (dot_entry[0] != 0x00 && dot_entry[0] != 0xE5 && 
                        (dot_entry[11] & 0x0F) != LFN_ATTR &&
                        !(dot_entry[11] & 0x08)) {  // Not volume label
                        
                        // Try to extract long filename
                        char long_filename[256];
                        if (extract_long_filename(sector_buffer, lfn_start, lfn_start + lfn_count - 1, 
                                                 long_filename, sizeof(long_filename))) {
                            
                            // Extract file size and attributes from 8.3 entry
                            uint32_t file_size = (dot_entry[31] << 24) | (dot_entry[30] << 16) | 
                                               (dot_entry[29] << 8) | dot_entry[28];
                            
                            uint8_t attr = dot_entry[11];
                            // Skip hidden and system files
                            if (attr & 0x02 || attr & 0x04 || attr & 0x10) {
                                continue;  // Skip hidden, directory and system files
                            }
                            
                            printf("%-28s %10lu\n", long_filename, file_size);
                            entries_found++;
                            
                            // Skip past all the LFN entries and the 8.3 entry
                            entry_idx = scan_idx;
                            continue;
                        }
                    }
                }
                
                // If LFN extraction failed, fall through to normal processing
            }
            
            // Skip volume labels and remaining long filename entries
            if (entry[11] & 0x0F) {
                continue;  // Long filename entry
            }
            if (entry[11] & 0x08) {
                continue;  // Volume label
            }
            
            // Extract standard 8.3 filename
            char filename[13];
            int pos = 0;
            
            // Copy name part (8 characters max)
            for (int i = 0; i < 8 && entry[i] != ' '; i++) {
                filename[pos++] = tolower(entry[i]);
            }
            
            // Add extension if present
            bool has_ext = false;
            for (int i = 8; i < 11; i++) {
                if (entry[i] != ' ') {
                    if (!has_ext) {
                        filename[pos++] = '.';
                        has_ext = true;
                    }
                    filename[pos++] = tolower(entry[i]);
                }
            }
            filename[pos] = '\0';
            
            // Extract file size
            uint32_t file_size = (entry[31] << 24) | (entry[30] << 16) | 
                               (entry[29] << 8) | entry[28];
            
            // Extract attributes
            uint8_t attr = entry[11];
            // Skip hidden, system, and directory entries
            if (attr & 0x02 || attr & 0x04 || attr & 0x10) {
                continue;  // Skip hidden, system and directory files
            }
            
            printf("%-28s %10lu\n", filename, file_size);
            entries_found++;
            
            // Stop if we've reached the maximum entries for FAT16/12
            if (fs_type != FS_TYPE_FAT32 && entries_found >= max_entries) {
                printf("\nTotal: %lu files found\n", entries_found);
                sd_release();
                return SD_OK;
            }
        }
    }
    
    printf("\nTotal: %lu files found\n", entries_found);
    sd_release();
    return SD_OK;
}
