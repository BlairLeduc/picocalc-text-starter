//
//  FatFs disk layer for the PicoCalc SD card
//
//  FatFs calls these functions to reach the card; they pass straight
//  through to the block functions in sdcard.c. There is one drive, the
//  SD card, numbered 0.
//

#include "pico/stdlib.h"

#include "ff.h"
#include "diskio.h"

#include "sdcard.h"

// STA_NOINIT until the card has been set up. FatFs checks it on every call
// and sets the card up again, and mounts it afresh, when it is set.
static volatile DSTATUS status = STA_NOINIT;

// Timer for SD card detection
static repeating_timer_t sd_card_detect_timer;

// A card that is pulled out, even if another goes in before the next file
// call, must be set up and mounted again, so mark it as soon as it goes.
static bool on_sd_card_detect(repeating_timer_t *rt)
{
    if (!sd_card_present())
    {
        status = STA_NOINIT | STA_NODISK;
    }
    return true;
}

void disk_detect_init(void)
{
    add_repeating_timer_ms(500, on_sd_card_detect, NULL, &sd_card_detect_timer);
}

DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != 0)
    {
        return STA_NOINIT;
    }

    if (!sd_card_present())
    {
        status = STA_NOINIT | STA_NODISK;
    }
    return status;
}

DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != 0)
    {
        return STA_NOINIT;
    }

    if (!sd_card_present())
    {
        status = STA_NOINIT | STA_NODISK;
    }
    else
    {
        status = sd_card_init() == SD_OK ? 0 : STA_NOINIT;
    }
    return status;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != 0 || count == 0)
    {
        return RES_PARERR;
    }
    if (status & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    return sd_read_blocks(sector, count, buff) == SD_OK ? RES_OK : RES_ERROR;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != 0 || count == 0)
    {
        return RES_PARERR;
    }
    if (status & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    return sd_write_blocks(sector, count, buff) == SD_OK ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv != 0)
    {
        return RES_PARERR;
    }
    if (status & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    // A write is finished on the card before sd_write_blocks returns, so
    // there is nothing to flush. The other commands are for f_mkfs and
    // trim, which are not turned on in ffconf.h.
    return cmd == CTRL_SYNC ? RES_OK : RES_PARERR;
}
