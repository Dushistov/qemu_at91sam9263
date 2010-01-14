/*
 *  CFI parallel flash for emulating Atmel NOR flash (AT49BV642D)
 *
 *  Copyright (c) 2009 Evgeniy Dushistov
 *  Copyright (c) 2006 Thorsten Zitterell
 *  Copyright (c) 2005 Jocelyn Mayer
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

/*
 * For now, this code can emulate flashes of 1, 2 or 4 bytes width.
 * Supported commands/modes are:
 * - flash read
 * - flash write
 * - flash ID read
 * - sector erase
 *
 * It does not support timings
 * It does not support flash interleaving
 * It does not implement software data protection as found in many real chips
 * It does not implement erase suspend/resume commands
 * It does not implement multiple sectors erase
 *
 * It does not implement much more ...
 */

#include "hw.h"
#include "flash.h"
#include "block.h"
#include "qemu-timer.h"

#define PFLASH_BUG(fmt, ...)                                    \
    do {                                                        \
        printf("PFLASH: Possible BUG - " fmt, ## __VA_ARGS__);  \
        exit(1);                                                \
    } while(0)

//#define PFLASH_DEBUG
#ifdef PFLASH_DEBUG
#define DPRINTF(fmt, ...)                           \
    do {                                            \
        printf("PFLASH: " fmt , ## __VA_ARGS__);    \
    } while (0)
#else
#define DPRINTF(fmt, ...) do { } while (0)
#endif

struct pflash_t {
    BlockDriverState *bs;
    target_phys_addr_t base;
    target_phys_addr_t sector_len;
    target_phys_addr_t boot_sect_len;
    target_phys_addr_t nb_blocks;
    target_phys_addr_t nb_boot_blocks;
    target_phys_addr_t total_len;
    int width;
    int wcycle; /* if 0, the flash is read normally */
    int bypass;
    int ro;
    uint8_t cmd;
    uint8_t status;
    uint16_t ident[4];
    target_phys_addr_t counter;
    QEMUTimer *timer;
    ram_addr_t off;
    int fl_mem;
    void *storage;
};

#define AT49_FLASH_CODE1        0xAA
#define AT49_FLASH_CODE2        0x55
#define AT49_ID_IN_CODE        0x90
#define AT49_ID_OUT_CODE        0xF0
#define AT49_FLASH_Setup_Erase     0x80
#define AT49_FLASH_Program         0xA0
#define AT49_FLASH_Sector_Erase    0x30
#define AT49_CMD_READ_ARRAY        0x00F0
#define AT49_MEM_FLASH_ADDR1 (0x5555<<1)
#define AT49_MEM_FLASH_ADDR2 (0x00002AAA<<1)

static void pflash_timer (void *opaque)
{
    pflash_t *pfl = opaque;

    DPRINTF("%s: command %02x done\n", __func__, pfl->cmd);
    /* Reset flash */
    pfl->status ^= 0x80;
    if (pfl->bypass) {
        pfl->wcycle = 2;
    } else {
        cpu_register_physical_memory(pfl->base, pfl->total_len,
                                     pfl->off | IO_MEM_ROMD | pfl->fl_mem);
        pfl->wcycle = 0;
    }
    pfl->cmd = 0;
}

static uint32_t pflash_read (pflash_t *pfl, target_phys_addr_t offset,
                             int width)
{
    target_phys_addr_t boff;
    uint32_t ret;
    uint8_t *p;

    ret = -1;
    boff = offset & 0xFF; /* why this here ?? */

    if (pfl->width == 2)
        boff = boff >> 1;
    else if (pfl->width == 4)
        boff = boff >> 2;

    DPRINTF("%s: reading offset " TARGET_FMT_plx " under cmd %02x width %d\n",
            __func__, offset, pfl->cmd, width);

    switch (pfl->cmd) {
    case 0x00:
        /* Flash area read */
        p = pfl->storage;
        switch (width) {
        case 1:
            ret = p[offset];
            DPRINTF("%s: data offset " TARGET_FMT_plx " %02x\n",
                    __func__, offset, ret);
            break;
        case 2:
#if defined(TARGET_WORDS_BIGENDIAN)
            ret = p[offset] << 8;
            ret |= p[offset + 1];
#else
            ret = p[offset];
            ret |= p[offset + 1] << 8;
#endif
            DPRINTF("%s: data offset " TARGET_FMT_plx " %04x\n",
                    __func__, offset, ret);
            break;
        case 4:
#if defined(TARGET_WORDS_BIGENDIAN)
            ret = p[offset] << 24;
            ret |= p[offset + 1] << 16;
            ret |= p[offset + 2] << 8;
            ret |= p[offset + 3];
#else
            ret = p[offset];
            ret |= p[offset + 1] << 8;
            ret |= p[offset + 1] << 8;
            ret |= p[offset + 2] << 16;
            ret |= p[offset + 3] << 24;
#endif
            DPRINTF("%s: data offset " TARGET_FMT_plx " %08x\n",
                    __func__, offset, ret);
            break;
        default:
            DPRINTF("BUG in %s\n", __func__);
        }

        break;
    case AT49_FLASH_Setup_Erase: /* Block erase */
    case AT49_FLASH_Sector_Erase:
    case AT49_FLASH_Program: /* Write block */
        /* Status register read */
        ret = pfl->status;
        DPRINTF("%s: status %x\n", __func__, ret);
        break;
    case AT49_ID_IN_CODE:
        DPRINTF("we read ID, boff %u, %X %X\n", boff, pfl->ident[0], pfl->ident[1]);
        /* flash ID read */
        switch (boff) {
        case 0x00:
        case 0x01:
            ret = pfl->ident[boff & 0x01];
            break;
        case 0x02:
            ret = 0x00; /* Pretend all sectors are unprotected */
            break;
        default:
            break;
        }
        break;
    default:
        /* This should never happen : reset state & treat it as a read */
        DPRINTF("%s: unknown command state: %x\n", __func__, pfl->cmd);
        pfl->wcycle = 0;
        pfl->cmd = 0;
    }
    DPRINTF("%s: ret %X\n", __func__, ret);
    return ret;
}

/* update flash content on disk */
static void pflash_update(pflash_t *pfl, int offset,
                          int size)
{
    int offset_end;
    if (pfl->bs) {
        offset_end = offset + size;
        /* round to sectors */
        offset = offset >> 9;
        offset_end = (offset_end + 511) >> 9;
        bdrv_write(pfl->bs, offset, pfl->storage + (offset << 9),
                   offset_end - offset);
    }
}

static inline void pflash_data_write(pflash_t *pfl, target_phys_addr_t offset,
                                     uint32_t value, int width)
{
    uint8_t *p = pfl->storage;

    DPRINTF("%s: block write offset " TARGET_FMT_plx
            " value %x counter " TARGET_FMT_plx "\n",
            __func__, offset, value, pfl->counter);
    switch (width) {
    case 1:
        p[offset] = value;
        pflash_update(pfl, offset, 1);
        break;
    case 2:
#if defined(TARGET_WORDS_BIGENDIAN)
        p[offset] = value >> 8;
        p[offset + 1] = value;
#else
        p[offset] = value;
        p[offset + 1] = value >> 8;
#endif
        pflash_update(pfl, offset, 2);
        break;
    case 4:
#if defined(TARGET_WORDS_BIGENDIAN)
        p[offset] = value >> 24;
        p[offset + 1] = value >> 16;
        p[offset + 2] = value >> 8;
        p[offset + 3] = value;
#else
        p[offset] = value;
        p[offset + 1] = value >> 8;
        p[offset + 2] = value >> 16;
        p[offset + 3] = value >> 24;
#endif
        pflash_update(pfl, offset, 4);
        break;
    }

}

static void pflash_write(pflash_t *pfl, target_phys_addr_t offset,
                         uint32_t value, int width)
{
    target_phys_addr_t boff;
    uint8_t *p;
    uint8_t cmd;

    cmd = value;

    DPRINTF("%s: writing offset " TARGET_FMT_plx " value %08x width %d wcycle 0x%x\n",
            __func__, offset, value, width, pfl->wcycle);

    /* Set the device in I/O access mode */
    cpu_register_physical_memory(pfl->base, pfl->total_len, pfl->fl_mem);
    boff = offset & (pfl->sector_len - 1);

    if (pfl->width == 2)
        boff = boff >> 1;
    else if (pfl->width == 4)
        boff = boff >> 2;

    switch (pfl->wcycle) {
    case 0:
        /* read mode */
        switch (cmd) {
        case AT49_CMD_READ_ARRAY:
            if (pfl->cmd == AT49_FLASH_Program) {
                DPRINTF("%s: finish program\n", __func__);
                goto reset_flash;
            }
            goto error_flash;
        case AT49_FLASH_CODE1:
            if (offset == AT49_MEM_FLASH_ADDR1) {
                DPRINTF("flash code 1\n");
                break;
            }
        default:
            goto error_flash;
        }
        pfl->wcycle++;
        pfl->cmd = cmd;
        return;
    case 1:
        DPRINTF("offset %X, cmd %X wcyle 1\n", offset, pfl->cmd);
        switch (pfl->cmd) {
        case AT49_FLASH_Program:
            DPRINTF("%s: Single Byte Program\n", __func__);
            pflash_data_write(pfl, offset, value, width);
            /* pfl->status = (value & 0x80); /\* Ready! *\/ */
            pfl->wcycle = 0;
            goto reset_flash;
        case AT49_FLASH_CODE1:
            if (offset == AT49_MEM_FLASH_ADDR2 && value == AT49_FLASH_CODE2) {
                DPRINTF("flash code 2\n");
                pfl->cmd = value;
                pfl->wcycle++;
                break;
            }
        default:
            goto error_flash;
        }
        return;
    case 2:
        DPRINTF("%s: cmd %X\n", __func__, pfl->cmd);
        switch (pfl->cmd) {
        case AT49_FLASH_Sector_Erase:
            if (value == AT49_CMD_READ_ARRAY) {
                DPRINTF("%s: sector erase finished\n", __func__);
                goto reset_flash;
            }
            break;
        case AT49_FLASH_CODE2:
            if (offset == AT49_MEM_FLASH_ADDR1) {
                switch (value) {
                case AT49_ID_IN_CODE:
                    DPRINTF("%s: ident command\n", __func__);
                    pfl->wcycle = 0;
                    break;
                case AT49_FLASH_Setup_Erase:
                    DPRINTF("%s: erase command\n", __func__);
                    pfl->wcycle = 0;
                    break;
                case AT49_FLASH_Program:
                    DPRINTF("%s: program command\n", __func__);
                    pfl->wcycle = 1;
                    break;
                case AT49_ID_OUT_CODE:
                    DPRINTF("%s: ident command finished\n", __func__);
                    goto reset_flash;
                default:
                    goto error_flash;
                }
                pfl->cmd = value;
                break;
            } else if (value == AT49_FLASH_Sector_Erase) {
                target_phys_addr_t sect_len = offset < pfl->boot_sect_len * pfl->nb_boot_blocks ?
                    pfl->boot_sect_len : pfl->sector_len;
                pfl->cmd = AT49_FLASH_Sector_Erase;
                p = pfl->storage;
                DPRINTF("%s: offset " TARGET_FMT_plx "\n", __func__, offset);
                offset &= ~(sect_len - 1);
                DPRINTF("%s: block erase at " TARGET_FMT_plx " bytes "
                        TARGET_FMT_plx "\n",
                        __func__, offset, sect_len);

                memset(p + offset, 0xff, sect_len);
                pflash_update(pfl, offset, sect_len);
                pfl->wcycle = 0;
                pfl->status |= 0x80; /* Ready! */
                break;
            }
        default:
            goto error_flash;
        }
        return;
    default:
        /* Should never happen */
        DPRINTF("%s: invalid write state\n",  __func__);
        goto reset_flash;
    }
    return;

error_flash:
    printf("%s: Unimplemented flash cmd sequence "
           "(offset " TARGET_FMT_plx ", wcycle 0x%x cmd 0x%x value 0x%x)\n",
           __func__, offset, pfl->wcycle, pfl->cmd, value);

reset_flash:
    cpu_register_physical_memory(pfl->base, pfl->total_len,
                                 pfl->off | IO_MEM_ROMD | pfl->fl_mem);

    pfl->bypass = 0;
    pfl->wcycle = 0;
    pfl->cmd = 0;
    return;
}


static uint32_t pflash_readb (void *opaque, target_phys_addr_t addr)
{
    return pflash_read(opaque, addr, 1);
}

static uint32_t pflash_readw (void *opaque, target_phys_addr_t addr)
{
    pflash_t *pfl = opaque;

    return pflash_read(pfl, addr, 2);
}

static uint32_t pflash_readl (void *opaque, target_phys_addr_t addr)
{
    pflash_t *pfl = opaque;

    return pflash_read(pfl, addr, 4);
}

static void pflash_writeb (void *opaque, target_phys_addr_t addr,
                           uint32_t value)
{
    pflash_write(opaque, addr, value, 1);
}

static void pflash_writew (void *opaque, target_phys_addr_t addr,
                           uint32_t value)
{
    pflash_t *pfl = opaque;

    pflash_write(pfl, addr, value, 2);
}

static void pflash_writel (void *opaque, target_phys_addr_t addr,
                           uint32_t value)
{
    pflash_t *pfl = opaque;

    pflash_write(pfl, addr, value, 4);
}

static CPUWriteMemoryFunc *  pflash_write_ops[] = {
    &pflash_writeb,
    &pflash_writew,
    &pflash_writel,
};

static CPUReadMemoryFunc * pflash_read_ops[] = {
    &pflash_readb,
    &pflash_readw,
    &pflash_readl,
};

pflash_t *pflash_cfi_atmel_register(target_phys_addr_t base, ram_addr_t off,
                                    BlockDriverState *bs,
                                    uint32_t boot_sect_len,
                                    int nb_boot_blocks,
                                    uint32_t sector_len,
                                    int nb_blocs, int width,
                                    uint16_t id0, uint16_t id1,
                                    uint16_t id2, uint16_t id3)
{
    pflash_t *pfl;
    target_phys_addr_t total_len;
    int ret;

    total_len = boot_sect_len * nb_boot_blocks + sector_len * nb_blocs;

    pfl = qemu_mallocz(sizeof(pflash_t));

    pfl->storage = qemu_get_ram_ptr(off);
    pfl->fl_mem = cpu_register_io_memory(
        pflash_read_ops, pflash_write_ops, pfl);
    pfl->off = off;
    cpu_register_physical_memory(base, total_len,
                                 off | pfl->fl_mem | IO_MEM_ROMD);

    pfl->bs = bs;
    if (pfl->bs) {
        /* read the initial flash content */
        ret = bdrv_read(pfl->bs, 0, pfl->storage, total_len >> 9);
        if (ret < 0) {
            cpu_unregister_io_memory(pfl->fl_mem);
            qemu_free(pfl);
            return NULL;
        }
    }
    pfl->ro = 0;
    pfl->timer = qemu_new_timer(vm_clock, pflash_timer, pfl);
    pfl->base = base;
    pfl->sector_len = sector_len;
    pfl->nb_blocks = nb_blocs;
    pfl->boot_sect_len = boot_sect_len;
    pfl->nb_boot_blocks = nb_boot_blocks;
    pfl->total_len = total_len;
    pfl->width = width;
    pfl->wcycle = 0;
    pfl->cmd = 0;
    pfl->status = 0;
    pfl->ident[0] = id0;
    pfl->ident[1] = id1;
    pfl->ident[2] = id2;
    pfl->ident[3] = id3;


    return pfl;
}
