//
//  PicoCalc SD Card driver for FAT32 formatted SD cards
//
//  This driver provides block-level access to SD cards and implements
//  basic FAT32 file system operations for reading and writing files.
//
//  Only Master Boot Record (MBR) disk layout is supported (not GPT).
//  FAT32 without the MBR partition table is supported.
//  Standard SD cards (SDSC) and SD High Capacity (SDHC) cards are supported.
//

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <strings.h> // For strcasecmp

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico/sem.h"

#include "sdcard.h"
#include "fat32.h"

// Global state
static bool fat32_mounted = false;
static fat32_error_t fat32_status = FAT32_OK; // Error code for mount operation
bool fat32_initialised = false; // Set to true after successful file system initialization 

// FAT32 file system state
static fat32_boot_sector_t boot_sector;
static uint32_t volume_start_block = 0; // First block of the volume
static uint32_t first_data_sector;      // First sector of the data region
static uint32_t data_region_sectors;    // Total sectors in the data region
static uint32_t cluster_count;          // Total number of clusters in the data region
static uint32_t bytes_per_cluster;

static uint32_t current_dir_cluster = 0; // Current directory cluster

// Working buffers
static uint8_t sector_buffer[FAT32_SECTOR_SIZE] __attribute__((aligned(4)));

// Timer for SD card detection
static repeating_timer_t sd_card_detect_timer;

//
//  Sector-level access functions
//

static uint32_t cluster_to_sector(uint32_t cluster)
{
    return ((cluster - 2) * boot_sector.sectors_per_cluster) + first_data_sector;
}

static fat32_error_t fat32_read_sector(uint32_t sector, uint8_t *buffer)
{
    return sd_read_block(volume_start_block + sector, buffer);
}

static fat32_error_t fat32_write_sector(uint32_t sector, const uint8_t *buffer)
{
    return sd_write_block(volume_start_block + sector, buffer);
}

//
// FAT32 file system functions
//

static bool fat32_is_mbr(const uint8_t *sector)
{
    // Check for 0x55AA signature
    if (sector[510] != 0x55 || sector[511] != 0xAA)
    {
        return false;
    }

    // Check for valid partition type in partition table
    for (int i = 0; i < 4; i++)
    {
        uint8_t part_type = sector[446 + i * 16 + 4];
        if (part_type != 0x00)
        {
            return true; // At least one valid partition
        }
    }
    return false;
}

static bool fat32_is_fat_boot_sector(const uint8_t *sector)
{
    // Check for 0x55AA signature
    if (sector[510] != 0x55 || sector[511] != 0xAA)
    {
        return false;
    }

    // Check for valid jump instruction
    if (sector[0] != 0xEB && sector[0] != 0xE9)
    {
        return false;
    }

    // Check for reasonable bytes per sector (should be 512, 1024, 2048, or 4096)
    uint16_t bps = sector[11] | (sector[12] << 8);
    if (bps != 512 && bps != 1024 && bps != 2048 && bps != 4096)
    {
        return false;
    }

    return true;
}

static fat32_error_t is_valid_fat32_boot_sector(const fat32_boot_sector_t *bs)
{
    // Check bytes per sector - this is critical
    if (bs->bytes_per_sector != FAT32_SECTOR_SIZE)
    {
        return FAT32_ERROR_INVALID_FORMAT;
    }

    // Check sectors per cluster (must be power of 2)
    uint8_t spc = bs->sectors_per_cluster;
    if (spc == 0 || spc > 128 || (spc & (spc - 1)) != 0)
    {
        return FAT32_ERROR_INVALID_FORMAT;
    }

    // Check number of FATs
    if (bs->num_fats == 0 || bs->num_fats > 2)
    {
        return FAT32_ERROR_INVALID_FORMAT;
    }

    // Check if we have valid reserved sectors
    if (bs->reserved_sectors == 0)
    {
        return FAT32_ERROR_INVALID_FORMAT;
    }

    // Check if fat size is valid
    if (bs->fat_size_16 != 0 || bs->fat_size_32 == 0)
    {
        return FAT32_ERROR_INVALID_FORMAT;
    }

    if (bs->total_sectors_32 == 0)
    {
        return FAT32_ERROR_INVALID_FORMAT;
    }

    return FAT32_OK;
}

static fat32_error_t read_cluster_fat_entry(uint32_t cluster, uint32_t *value)
{
    // The first data cluster is 2, less than 2 is invalid
    if (cluster < 2)
    {
        return FAT32_ERROR_INVALID_PARAMETER;
    }

    uint32_t fat_offset = cluster * 4; // 4 bytes per entry in FAT32
    uint32_t fat_sector = boot_sector.reserved_sectors + (fat_offset / FAT32_SECTOR_SIZE);
    uint32_t entry_offset = fat_offset % FAT32_SECTOR_SIZE;

    // Read the FAT sector
    fat32_error_t result = fat32_read_sector(fat_sector, sector_buffer);
    if (result != FAT32_OK)
    {
        return result;
    }

    uint32_t entry = *(uint32_t *)(sector_buffer + entry_offset);
    *value = entry & 0x0FFFFFFF; // Mask out upper 4 bits for FAT32
    return FAT32_OK;
}

static fat32_error_t write_cluster_fat_entry(uint32_t cluster, uint32_t value)
{
    // The first data cluster is 2, less than 2 is invalid
    if (cluster < 2)
    {
        return FAT32_ERROR_INVALID_PARAMETER;
    }

    uint32_t fat_offset = cluster * 4; // 4 bytes per entry in FAT32
    uint32_t fat_sector = boot_sector.reserved_sectors + (fat_offset / FAT32_SECTOR_SIZE);
    uint32_t entry_offset = fat_offset % FAT32_SECTOR_SIZE;

    // Read the FAT sector
    fat32_error_t result = fat32_read_sector(fat_sector, sector_buffer);
    if (result != FAT32_OK)
    {
        return result;
    }

    // Write the FAT entry
    *(uint32_t *)(sector_buffer + entry_offset) &= 0xF0000000;
    *(uint32_t *)(sector_buffer + entry_offset) |= value & 0x0FFFFFFF;

    // Write the modified sector back
    result = fat32_write_sector(fat_sector, sector_buffer);
    if (result != FAT32_OK)
    {
        return result; // Error writing back
    }

    return FAT32_OK;
}

//
// Mount the SD Card functions
//

fat32_error_t fat32_mount(void)
{
    if (!sd_card_present())
    {
        fat32_unmount(); // Unmount if card is not present
        return FAT32_ERROR_NO_CARD;
    }

    if (fat32_mounted)
    {
        return FAT32_OK;
    }

    fat32_error_t result = sd_card_present();
    if (result != FAT32_OK)
    {
        return result;
    }

    // Read boot sector
    result = sd_read_block(0, sector_buffer);
    if (result != FAT32_OK)
    {
        return result;
    }

    // Is this a Master Boot Record (MBR)?
    if (fat32_is_mbr(sector_buffer))
    {
        volume_start_block = 0; // Set to zero to detect if no partitions are acceptable

        // Read partition table entries
        for (int i = 0; i < 4; i++)
        {
            // Read next partition table entry
            mbr_partition_entry_t *partition_entry = (mbr_partition_entry_t *)(sector_buffer + 446 + i * 16);

            // Check if this partition is active
            if (partition_entry->boot_indicator != 0x00 && partition_entry->boot_indicator != 0x80)
            {
                continue; // No partition here
            }
            if (partition_entry->partition_type == 0x0B || // FAT32 with CHS addressing
                partition_entry->partition_type == 0x0C)   // FAT32 with LBA addressing
            {
                // Align disk accesses with the partition we have decided to use
                volume_start_block = partition_entry->start_lba;

                // Read the boot sector from the partition
                result = sd_read_block(volume_start_block, sector_buffer);
                if (result != FAT32_OK)
                {
                    return result;
                }
                break;
            }
        }
        if (volume_start_block == 0)
        {
            return FAT32_ERROR_INVALID_FORMAT; // No valid FAT32 partition found
        }
    }
    else if (fat32_is_fat_boot_sector(sector_buffer))
    {
        // No partition table, treat the entire disk as a single partition
        volume_start_block = 0;

        // We already have the boot sector in sector_buffer, no fat32_read_block needed
    }
    else
    {
        return FAT32_ERROR_INVALID_FORMAT; // This is not a valid FAT32 boot sector
    }

    // Copy boot sector data
    memcpy(&boot_sector, sector_buffer, sizeof(fat32_boot_sector_t));

    // Validate boot sector
    result = is_valid_fat32_boot_sector(&boot_sector);
    if (result != FAT32_OK)
    {
        return result;
    }

    // Calculate important sectors/clusters
    bytes_per_cluster = boot_sector.sectors_per_cluster * FAT32_SECTOR_SIZE;
    first_data_sector = boot_sector.reserved_sectors + (boot_sector.num_fats * boot_sector.fat_size_32);
    data_region_sectors = boot_sector.total_sectors_32 - (boot_sector.num_fats * boot_sector.fat_size_32);
    cluster_count = data_region_sectors / boot_sector.sectors_per_cluster;
    if (cluster_count < 65525)
    {
        return FAT32_ERROR_INVALID_FORMAT; // This is FAT12 or FAT16, not FAT32!
    }

    current_dir_cluster = boot_sector.root_cluster; // Start at root directory
    fat32_mounted = true;
    return FAT32_OK;
}

void fat32_unmount(void)
{
    fat32_mounted = false;
    fat32_status = FAT32_OK; // Reset mount status
    volume_start_block = 0;
}

bool fat32_is_mounted(void)
{
    return fat32_mounted;
}

bool fat32_is_ready(void)
{
    return fat32_initialised && fat32_mounted && !sd_card_present();
}



fat32_error_t fat32_get_status(void)
{
    return fat32_status;
}

fat32_error_t fat32_get_free_space(uint64_t *free_space)
{
    // We can only get free space for FAT32 using FSInfo
    // Computing free space will be too slow for us

    if (!fat32_is_ready())
    {
        return fat32_status;
    }

    fat32_error_t result = fat32_read_sector(boot_sector.fat32_info, sector_buffer);
    if (result != FAT32_OK)
    {
        return result; // Error reading FSInfo sector
    }

    fat32_fsinfo_t *fsinfo = (fat32_fsinfo_t *)sector_buffer;

    // Verify FSInfo signatures
    if (fsinfo->lead_sig == 0x41615252 &&
        fsinfo->struc_sig == 0x61417272 &&
        fsinfo->trail_sig == 0xAA550000 &&
        fsinfo->free_count != 0xFFFFFFFF &&
        fsinfo->free_count <= cluster_count)
    {
        *free_space = ((uint64_t)fsinfo->free_count) * bytes_per_cluster;
        return FAT32_OK; // Successfully retrieved free space
    }

    // If FSInfo is not valid, we will count free clusters manually
    uint64_t free_clusters = 0;
    for (uint32_t sector = 0; sector < boot_sector.fat_size_32; sector++)
    {
        result = fat32_read_sector(boot_sector.reserved_sectors + sector, sector_buffer);
        if (result != FAT32_OK)
        {
            return result; // Error reading FAT sector
        }
        for (int i = 0; i < FAT32_SECTOR_SIZE; i += 4)
        {
            uint32_t entry = *(uint32_t *)(sector_buffer + i) & 0x0FFFFFFF;
            if (entry == 0)
            {
                free_clusters++;
            }
        }
    }

    *free_space = free_clusters * bytes_per_cluster;
    return FAT32_OK;
}

fat32_error_t fat32_get_total_space(uint64_t *total_space)
{
    if (!fat32_is_ready())
    {
        return fat32_status;
    }

    // Get the total number of sectors
    uint64_t total_sectors = boot_sector.total_sectors_32;

    // Calculate total space in bytes
    *total_space = total_sectors * FAT32_SECTOR_SIZE;

    return FAT32_OK;
}

fat32_error_t fat32_get_volume_name(char *name, size_t name_len)
{
    if (!name || name_len < 12)
    {
        return FAT32_ERROR_INVALID_PARAMETER; // Name buffer too small
    }

    if (!fat32_is_ready())
    {
        return fat32_status;
    }

    // Read the volume label from the root directory
    fat32_dir_t dir = {0};
    dir.is_open = true;
    dir.start_cluster = boot_sector.root_cluster;
    dir.current_cluster = boot_sector.root_cluster;
    dir.position = 0;

    bool found = false;
    fat32_entry_t entry;
    while (fat32_dir_read(&dir, &entry) == FAT32_OK && entry.name[0])
    {
        if (entry.attr & FAT32_ATTR_VOLUME_ID)
        {
            // Found a volume label entry
            strncpy(name, entry.name, name_len - 1);
            name[name_len - 1] = '\0'; // Ensure null-termination
            return FAT32_OK;
        }
    }
    name[0] = '\0'; // No volume label found
    return FAT32_OK;
}

//
// File system naming utility functions
//

static void convert_filename_to_83(const char *filename, char *name83)
{
    memset(name83, ' ', 11);
    name83[11] = '\0';

    const char *dot = strrchr(filename, '.');
    int name_len = dot ? (dot - filename) : strlen(filename);

    // Copy name part (max 8 characters)
    for (int i = 0; i < name_len && i < 8; i++)
    {
        name83[i] = toupper(filename[i]);
    }

    // Copy extension part (max 3 characters)
    if (dot && strlen(dot + 1) > 0)
    {
        for (int i = 0; i < 3 && dot[1 + i]; i++)
        {
            name83[8 + i] = toupper(dot[1 + i]);
        }
    }
}

static void convert_83_to_filename(const char *name83, char *filename)
{
    int pos = 0;

    // Copy name part
    for (int i = 0; i < 8 && name83[i] != ' '; i++)
    {
        filename[pos++] = tolower(name83[i]);
    }

    // Copy extension part
    bool has_ext = false;
    for (int i = 8; i < 11; i++)
    {
        if (name83[i] != ' ')
        {
            if (!has_ext)
            {
                filename[pos++] = '.';
                has_ext = true;
            }
            filename[pos++] = tolower(name83[i]);
        }
    }

    filename[pos] = '\0';
}

static inline char utf16_to_utf8(uint16_t utf16)
{
    // Convert UTF-16 to UTF-8 (simplified - only handles ASCII range)
    return utf16 < 0x80 ? (char)utf16 : '?';
}

static uint8_t lfn_checksum(const char *name83)
{
    // Calculate checksum for 8.3 filename
    uint8_t sum = 0;
    for (uint8_t i = 11; i > 0; i--)
    {
        // NOTE: The operation is an unsigned char rotate right
        sum = ((sum & 1) ? 0x80 : 0) + (sum >> 1) + (uint8_t)*name83++;
    }
    return sum;
}

static void lfn_entry_into_buffer(fat32_lfn_entry_t *lfn_entry, char *buffer)
{
    // Convert UTF-16 parts to UTF-8
    *(buffer++) = utf16_to_utf8(lfn_entry->name1[0]);
    *(buffer++) = utf16_to_utf8(lfn_entry->name1[1]);
    *(buffer++) = utf16_to_utf8(lfn_entry->name1[2]);
    *(buffer++) = utf16_to_utf8(lfn_entry->name1[3]);
    *(buffer++) = utf16_to_utf8(lfn_entry->name1[4]);
    *(buffer++) = utf16_to_utf8(lfn_entry->name2[0]);
    *(buffer++) = utf16_to_utf8(lfn_entry->name2[1]);
    *(buffer++) = utf16_to_utf8(lfn_entry->name2[2]);
    *(buffer++) = utf16_to_utf8(lfn_entry->name2[3]);
    *(buffer++) = utf16_to_utf8(lfn_entry->name2[4]);
    *(buffer++) = utf16_to_utf8(lfn_entry->name2[5]);
    *(buffer++) = utf16_to_utf8(lfn_entry->name3[0]);
    *(buffer++) = utf16_to_utf8(lfn_entry->name3[1]);
}

static fat32_error_t find_directory_entry(fat32_entry_t *dir_entry, const char *path)
{
    if (!dir_entry || !path || !*path)
    {
        return FAT32_ERROR_INVALID_PARAMETER;
    }

    // Determine starting cluster: root or current
    uint32_t cluster = current_dir_cluster;
    if (path[0] == '/')
    {
        cluster = boot_sector.root_cluster;
    }

    // Copy path and tokenize
    char path_copy[FAT32_MAX_PATH_LEN];
    strncpy(path_copy, path + (path[0] == '/' ? 1 : 0), sizeof(path_copy) - 1);
    path_copy[sizeof(path_copy) - 1] = '\0';

    char *saveptr = NULL;
    char *token = strtok_r(path_copy, "/", &saveptr);
    char *next_token = NULL;

    while (token)
    {
        next_token = strtok_r(NULL, "/", &saveptr);

        // Open the current directory cluster
        fat32_dir_t dir = {0};
        dir.is_open = true;
        dir.start_cluster = cluster;
        dir.current_cluster = cluster;
        dir.position = 0;

        bool found = false;
        fat32_entry_t entry;
        while (fat32_dir_read(&dir, &entry) == FAT32_OK && entry.name[0])
        {
            if (strcasecmp(entry.name, token) == 0)
            {
                // If this is the last component, return the entry
                if (!next_token)
                {
                    memcpy(dir_entry, &entry, sizeof(fat32_dir_entry_t));
                    fat32_dir_close(&dir);
                    return FAT32_OK;
                }
                // If not last, must be a directory
                if (entry.attr & FAT32_ATTR_DIRECTORY)
                {
                    cluster = entry.start_cluster ? entry.start_cluster : boot_sector.root_cluster;
                    found = true;
                    break;
                }
            }
        }
        fat32_dir_close(&dir);
        if (!found && next_token)
        {
            return FAT32_ERROR_INVALID_PATH; // Intermediate directory not found
        }
        token = next_token;
    }

    return FAT32_ERROR_FILE_NOT_FOUND; // Not found
}

//
// File operations (simplified implementation)
//

fat32_error_t fat32_file_open(fat32_file_t *file, const char *filename)
{
    if (!file || !filename)
    {
        return FAT32_ERROR_INVALID_PARAMETER;
    }

    if (!fat32_is_ready())
    {
        return fat32_status;
    }

    memset(file, 0, sizeof(fat32_file_t));

    fat32_entry_t entry;
    fat32_error_t result = find_directory_entry(&entry, filename);
    if (result != FAT32_OK)
    {
        return result; // File not found or error
    }
    if ((entry.attr & FAT32_ATTR_DIRECTORY) || (entry.attr & FAT32_ATTR_VOLUME_ID))
    {
        return FAT32_ERROR_NOT_A_FILE; // Not a valid file
    }

    // Found the file
    file->is_open = true;
    file->start_cluster = entry.start_cluster;
    file->current_cluster = file->start_cluster;
    file->file_size = entry.size;
    file->position = 0;
    file->attributes = entry.attr;

    return FAT32_OK;
}

fat32_error_t fat32_file_create(fat32_file_t *file, const char *filename)
{
    // This is a placeholder implementation
    // Creating files requires finding free directory entries and clusters
    return FAT32_ERROR_INVALID_PARAMETER;
}

fat32_error_t fat32_file_close(fat32_file_t *file)
{
    if (file && file->is_open)
    {
        memset(file, 0, sizeof(fat32_file_t));
    }

    return FAT32_OK;
}

fat32_error_t fat32_file_read(fat32_file_t *file, void *buffer, size_t size, size_t *bytes_read)
{
    if (!file || !file->is_open || !buffer)
    {
        return FAT32_ERROR_INVALID_PARAMETER;
    }

    if (!fat32_is_ready())
    {
        return fat32_status;
    }

    if (bytes_read)
    {
        *bytes_read = 0;
    }

    if (file->position >= file->file_size)
    {
        return FAT32_OK; // EOF
    }

    size_t remaining = file->file_size - file->position;
    if (size > remaining)
    {
        size = remaining;
    }

    size_t total_read = 0;
    uint8_t *dest = (uint8_t *)buffer;

    while (total_read < size)
    {
        uint32_t cluster_offset = file->position % bytes_per_cluster;
        uint32_t sector_in_cluster = cluster_offset / FAT32_SECTOR_SIZE;
        uint32_t byte_in_sector = cluster_offset % FAT32_SECTOR_SIZE;

        uint32_t sector = cluster_to_sector(file->current_cluster) + sector_in_cluster;

        fat32_error_t result = fat32_read_sector(sector, sector_buffer);
        if (result != FAT32_OK)
        {
            return result;
        }

        size_t bytes_to_copy = FAT32_SECTOR_SIZE - byte_in_sector;
        if (bytes_to_copy > size - total_read)
        {
            bytes_to_copy = size - total_read;
        }

        memcpy(dest + total_read, sector_buffer + byte_in_sector, bytes_to_copy);
        total_read += bytes_to_copy;
        file->position += bytes_to_copy;

        // Check if we need to move to the next cluster
        if ((file->position % bytes_per_cluster) == 0 && total_read < size)
        {
            uint32_t next_cluster;
            fat32_error_t fat_result = read_cluster_fat_entry(file->current_cluster, &next_cluster);
            if (fat_result != FAT32_OK || next_cluster >= FAT32_FAT_ENTRY_EOC)
            {
                // End of cluster chain or error
                break;
            }
            file->current_cluster = next_cluster;
        }
    }

    if (bytes_read)
    {
        *bytes_read = total_read;
    }
    return FAT32_OK;
}

fat32_error_t fat32_file_write(fat32_file_t *file, const void *buffer, size_t size)
{
    // This is a placeholder implementation
    // Writing files requires cluster allocation and FAT updates
    return FAT32_ERROR_INVALID_PARAMETER;
}

fat32_error_t fat32_file_seek(fat32_file_t *file, uint32_t position)
{
    if (!file || !file->is_open)
    {
        return FAT32_ERROR_INVALID_PARAMETER;
    }

    if (position > file->file_size)
    {
        position = file->file_size;
    }

    file->position = position;

    // Recalculate current cluster (simplified)
    uint32_t cluster_num = position / bytes_per_cluster;
    file->current_cluster = file->start_cluster + cluster_num;

    return FAT32_OK;
}

uint32_t fat32_file_tell(fat32_file_t *file)
{
    return file ? file->position : 0;
}

uint32_t fat32_file_size(fat32_file_t *file)
{
    return file ? file->file_size : 0;
}

bool fat32_file_eof(fat32_file_t *file)
{
    return file ? (file->position >= file->file_size) : true;
}

fat32_error_t fat32_file_delete(const char *filename)
{
    // This is a placeholder implementation
    return FAT32_ERROR_INVALID_PARAMETER;
}

//
// Directory operations (placeholder implementations)
//

fat32_error_t fat32_set_current_dir(const char *path)
{
    fat32_dir_t dir;
    fat32_error_t result = fat32_dir_open(&dir, path);
    if (result != FAT32_OK)
    {
        return result; // Directory not found or error
    }

    if (!dir.is_open)
    {
        return FAT32_ERROR_INVALID_PATH; // Directory not open
    }

    // Update current directory cluster and name
    current_dir_cluster = dir.start_cluster;
    fat32_dir_close(&dir); // Close the directory

    return FAT32_OK;
}

fat32_error_t fat32_get_current_dir(char *path, size_t path_len)
{
    if (!path || path_len < FAT32_MAX_PATH_LEN)
    {
        return FAT32_ERROR_INVALID_PARAMETER;
    }

    if (!fat32_is_ready())
    {
        return fat32_status;
    }

    // Special case: root
    if (current_dir_cluster == boot_sector.root_cluster)
    {
        strncpy(path, "/", path_len);
        path[path_len - 1] = '\0';
        return FAT32_OK;
    }

    // Walk up the tree, collecting names
    char components[16][FAT32_MAX_FILENAME_LEN + 1]; // Up to 16 levels deep
    int depth = 0;
    uint32_t cluster = current_dir_cluster;

    while (cluster != boot_sector.root_cluster && depth < 16)
    {
        // Open current directory and read ".." entry to get parent cluster
        fat32_dir_t dir = {0};
        dir.is_open = true;
        dir.start_cluster = cluster;
        dir.current_cluster = cluster;
        dir.position = 0;

        fat32_entry_t entry;
        uint32_t parent_cluster = boot_sector.root_cluster;
        int entry_count = 0;
        bool found_parent = false;

        // Find ".." entry (always the second entry)
        while (fat32_dir_read(&dir, &entry) == FAT32_OK && entry.name[0])
        {
            if ((entry.attr & FAT32_ATTR_DIRECTORY) && strcmp(entry.name, "..") == 0)
            {
                parent_cluster = entry.start_cluster ? entry.start_cluster : boot_sector.root_cluster;
                found_parent = true;
                break;
            }
            entry_count++;
            if (entry_count > 2)
            {
                break;
            }
        }
        fat32_dir_close(&dir);
        if (!found_parent)
        {
            break;
        }

        // Now, open parent directory and search for this cluster's name
        fat32_dir_t parent_dir = {0};
        parent_dir.is_open = true;
        parent_dir.start_cluster = parent_cluster;
        parent_dir.current_cluster = parent_cluster;
        parent_dir.position = 0;

        bool found_name = false;
        while (fat32_dir_read(&parent_dir, &entry) == FAT32_OK && entry.name[0])
        {
            if ((entry.attr & FAT32_ATTR_DIRECTORY) && entry.start_cluster == cluster &&
                strcmp(entry.name, ".") != 0 && strcmp(entry.name, "..") != 0)
            {
                strncpy(components[depth], entry.name, FAT32_MAX_FILENAME_LEN);
                components[depth][FAT32_MAX_FILENAME_LEN] = '\0';
                found_name = true;
                break;
            }
        }
        fat32_dir_close(&parent_dir);
        if (!found_name)
        {
            break;
        }
        cluster = parent_cluster;
        depth++;
    }

    // Build the path string
    path[0] = '\0';
    for (int i = depth - 1; i >= 0; i--)
    {
        strncat(path, "/", path_len - strlen(path) - 1);
        strncat(path, components[i], path_len - strlen(path) - 1);
    }
    if (path[0] == '\0')
    {
        strncpy(path, "/", path_len);
    }

    return FAT32_OK;
}

fat32_error_t fat32_dir_open(fat32_dir_t *dir, const char *path)
{
    if (!dir || !path)
    {
        return FAT32_ERROR_INVALID_PARAMETER;
    }

    if (!fat32_is_ready())
    {
        return fat32_status;
    }

    if (strlen(path) > FAT32_MAX_PATH_LEN)
    {
        return FAT32_ERROR_INVALID_PATH; // Path too long
    }

    memset(dir, 0, sizeof(fat32_dir_t));

    // Start at root
    uint32_t cluster = current_dir_cluster;

    // If the path is empty, or refers to the current or parent directory of the root directory
    if (path[0] == '\0' || ((strcmp(path, ".") == 0 || strcmp(path, "..") == 0) &&
                            current_dir_cluster == boot_sector.root_cluster))
    {
        // Special case: current directory or parent directory of the root directory
        dir->is_open = true;
        dir->start_cluster = current_dir_cluster;
        dir->current_cluster = current_dir_cluster;
        dir->position = 0;
        return FAT32_OK;
    }

    // If the path is absolute, start from root cluster
    if (path[0] == '/')
    {
        cluster = boot_sector.root_cluster;
    }

    // Parse path components
    char path_copy[FAT32_MAX_PATH_LEN];
    strncpy(path_copy, path + (path[0] == '/' ? 1 : 0), sizeof(path_copy) - 1); // Skip leading slash
    path_copy[sizeof(path_copy) - 1] = '\0';

    char *saveptr = NULL;
    char *token = strtok_r(path_copy, "/", &saveptr);
    while (token)
    {
        // Search for this directory entry in the current cluster
        fat32_dir_t temp_dir = {0};
        temp_dir.is_open = true;
        temp_dir.start_cluster = cluster;
        temp_dir.current_cluster = cluster;
        temp_dir.position = 0;

        fat32_entry_t entry;
        bool found = false;
        while (fat32_dir_read(&temp_dir, &entry) == FAT32_OK && entry.name[0])
        {
            if ((entry.attr & FAT32_ATTR_DIRECTORY) && strcasecmp(entry.name, token) == 0)
            {
                cluster = entry.start_cluster ? entry.start_cluster : boot_sector.root_cluster;
                found = true;
                break;
            }
        }
        if (!found)
        {
            return FAT32_ERROR_INVALID_PATH;
        }
        token = strtok_r(NULL, "/", &saveptr);
    }

    dir->is_open = true;
    dir->start_cluster = cluster;
    dir->current_cluster = cluster;
    dir->position = 0;
    return FAT32_OK;
}

fat32_error_t fat32_dir_read(fat32_dir_t *dir, fat32_entry_t *dir_entry)
{
    char long_filename[FAT32_MAX_FILENAME_LEN + 1];
    uint8_t expected_checksum = 0;
    uint32_t read_sector = 0xFFFFFFFF; // Invalid sector to start with

    if (!dir || !dir_entry)
    {
        return FAT32_ERROR_INVALID_PARAMETER;
    }

    if (!fat32_is_ready())
    {
        return fat32_status;
    }

    if (!dir->is_open)
    {
        return FAT32_ERROR_READ_FAILED;
    }

    memset(dir_entry, 0, sizeof(fat32_dir_entry_t));

    if (dir->last_entry_read)
    {
        // If we have already read the last entry, return end of directory
        return FAT32_OK;
    }

    long_filename[0] = '\0'; // Reset long filename buffer

    // Search through all directory sectors
    while (!dir->last_entry_read && dir_entry->name[0] == '\0')
    {
        uint32_t cluster_offset = dir->position % bytes_per_cluster;
        uint32_t sector_in_cluster = cluster_offset / FAT32_SECTOR_SIZE;
        uint32_t byte_in_sector = cluster_offset % FAT32_SECTOR_SIZE;

        uint32_t sector = cluster_to_sector(dir->current_cluster) + sector_in_cluster;

        if (sector != read_sector)
        {
            fat32_error_t result = fat32_read_sector(sector, sector_buffer);
            if (result != FAT32_OK)
            {
                return result;
            }
            read_sector = sector;
        }

        fat32_dir_entry_t *entry = (fat32_dir_entry_t *)(sector_buffer + dir->position % FAT32_SECTOR_SIZE);

        if (entry->name[0] == FAT32_DIR_ENTRY_END_MARKER)
        {
            // End of directory
            dir->last_entry_read = true; // Mark that we reached the end
        }
        else if (entry->attr == FAT32_ATTR_LONG_NAME)
        {
            // Populate long filename buffer with this entry's name contents
            fat32_lfn_entry_t *lfn_entry = (fat32_lfn_entry_t *)entry;
            if (lfn_entry->seq & 0x40)
            {
                // This is the last entry for the long filename and the first entry of the sequence
                // We are starting to build a new long filename, clear the long_filename buffer
                memset(long_filename, 0, sizeof(long_filename));
                expected_checksum = lfn_entry->checksum; // Save checksum for later comparison
            }

            if (lfn_entry->checksum == expected_checksum)
            {
                // Copy this entry's part of the long filename into the long_filename buffer
                int offset = ((lfn_entry->seq & 0x3F) - 1) * FAT32_DIR_LFN_PART_SIZE;
                lfn_entry_into_buffer(lfn_entry, long_filename + offset);
            }
        }
        else if (entry->name[0] != FAT32_DIR_ENTRY_FREE)
        {
            uint8_t checksum = lfn_checksum(entry->name);
            // Now check to see if this is the entry we are looking for
            if (long_filename[0] != '\0' && expected_checksum == checksum)
            {
                strcpy(dir_entry->name, long_filename);
            }
            else
            {
                convert_83_to_filename(entry->name, dir_entry->name);
            }
            dir_entry->attr = entry->attr;
            dir_entry->start_cluster = (entry->fst_clus_hi << 16) | entry->fst_clus_lo;
            dir_entry->size = entry->file_size;
            dir_entry->date = entry->wrt_date;
            dir_entry->time = entry->wrt_time;
        }

        dir->position += 32; // Move to next entry (32 bytes per entry)

        // Check if we need to move to the next cluster
        if ((dir->position % bytes_per_cluster) == 0)
        {
            uint32_t next_cluster;
            fat32_error_t result = read_cluster_fat_entry(dir->current_cluster, &next_cluster);
            if (result != FAT32_OK)
            {
                return result; // Error reading FAT entry
            }
            if (next_cluster >= FAT32_FAT_ENTRY_EOC)
            {
                // End of cluster chain
                dir->last_entry_read = true; // Mark that we reached the end
                return FAT32_OK;                // No more entries to read
            }
            dir->current_cluster = next_cluster;
        }
    }

    return FAT32_OK; // Successfully read a directory entry
}

fat32_error_t fat32_dir_close(fat32_dir_t *dir)
{
    if (dir && dir->is_open)
    {
        memset(dir, 0, sizeof(fat32_dir_t));
    }

    return FAT32_OK;
}

fat32_error_t fat32_dir_create(fat32_dir_t *dir, const char *dirname)
{
    return FAT32_ERROR_INVALID_PARAMETER;
}

fat32_error_t fat32_dir_delete(fat32_dir_t *dir, const char *dirname)
{
    return FAT32_ERROR_INVALID_PARAMETER;
}

const char *fat32_error_string(fat32_error_t error)
{
    switch (error)
    {
    case FAT32_OK:
        return "Success";
    case FAT32_ERROR_NO_CARD:
        return "No SD card present";
    case FAT32_ERROR_INIT_FAILED:
        return "SD card initialization failed";
    case FAT32_ERROR_READ_FAILED:
        return "Read operation failed";
    case FAT32_ERROR_WRITE_FAILED:
        return "Write operation failed";
    case FAT32_ERROR_INVALID_FORMAT:
        return "Invalid SD card format";
    case FAT32_ERROR_NOT_MOUNTED:
        return "File system not mounted";
    case FAT32_ERROR_FILE_NOT_FOUND:
        return "File not found";
    case FAT32_ERROR_INVALID_PATH:
        return "Invalid path";
    case FAT32_ERROR_NOT_A_DIRECTORY:
        return "Not a directory";
    case FAT32_ERROR_NOT_A_FILE:
        return "Not a file";
    case FAT32_ERROR_DIR_NOT_EMPTY:
        return "Directory not empty";
    case FAT32_ERROR_DIR_NOT_FOUND:
        return "Directory not found";
    case FAT32_ERROR_DISK_FULL:
        return "Disk full";
    case FAT32_ERROR_FILE_EXISTS:
        return "File already exists";
    case FAT32_ERROR_INVALID_PARAMETER:
        return "Invalid parameter";
    default:
        return "Unknown error";
    }
}


// Timer callback to check SD card presence and mount/unmount
bool on_sd_card_detect(repeating_timer_t *rt)
{
    static int delay_count = 2;

    // Check if SD card is present
    if (!sd_card_present() && fat32_is_mounted())
    {
        fat32_unmount();    // Unmount if card is not present
        delay_count = 2; // Reset retry count
        return true;     // Continue timer
    }

    if (sd_card_present() && delay_count > 0)
    {
        // If card is present but not mounted, wait a bit before trying to mount
        delay_count--;
        return true; // Continue timer
    }

    if (sd_card_present() && !fat32_is_mounted() && fat32_status == FAT32_OK && delay_count <= 0)
    {
        delay_count = 2; // Reset retry count after successful mount
        // Try to mount the SD card
        fat32_status = fat32_mount();
        if (fat32_status != FAT32_OK)
        {
            return true; // Continue timer
        }
    }

    return true;
}


void fat32_init(void)
{
    // Initialize the file system state
    fat32_mounted = false;
    fat32_status = FAT32_OK;
    volume_start_block = 0;
    first_data_sector = 0;
    data_region_sectors = 0;
    cluster_count = 0;
    bytes_per_cluster = 0;
    current_dir_cluster = 0;

        // Check if a SD card is present
    add_repeating_timer_ms(500, on_sd_card_detect, NULL, &sd_card_detect_timer);

}