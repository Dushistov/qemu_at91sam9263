#include "hw.h"
#include "block.h"
#include "spi.h"
#include "flash.h"

typedef struct SPIFlash {
    uint32_t cmd;
    unsigned cmd_len;
    uint32_t addr;
    void *storage;
    unsigned int len;
    BlockDriverState *bs;
    uint8_t *buf1;
} SPIFlash;

#define DF_PAGE_SIZE 528
#define BUF1_SIZE DF_PAGE_SIZE

//#define AT91_SPI_FLASH_DEBUG
#ifdef AT91_SPI_FLASH_DEBUG
#define DPRINTF(fmt, ...)                           \
    do {                                            \
        printf("AT91SPI_FLASH: " fmt , ## __VA_ARGS__);    \
    } while (0)
#else
#define DPRINTF(fmt, ...) do { } while (0)
#endif

static void spi_flash_reset_state(SPIFlash *s)
{
    s->cmd = 0;
    s->cmd_len = 0;
    s->addr = 0;
}

static void spi_flash_set_chipselect(void *opaque, int on)
{
    SPIFlash *s = opaque;

    DPRINTF("set NS %d\n", on);
    if (on == 0)
        spi_flash_reset_state(s);
}

static uint32_t spi_flash_txrx(void *opaque, uint32_t val, int len)
{
    SPIFlash *s = opaque;

    DPRINTF("txrx: val %X\n", val);
    switch (s->cmd) {
    case 0:
        DPRINTF("New cmd %X\n", val);
        s->cmd = val;        
        ++s->cmd_len;
        return 0;
    case 0xE8://TODO: replace magic constants
        ++s->cmd_len;
        if (s->cmd_len >= 2 && s->cmd_len <= 4) {
            s->addr |= (val & 0xFF) << ((4 - s->cmd_len) * 8);
        } else if (s->cmd_len >= 5 && s->cmd_len < (5 + 4)) {
            DPRINTF("addr is %X\n", s->addr);
            /*ignore bytes*/
        } else {
            uint8_t *bytes = s->storage;
            /*TODO: handle different page sizes*/
            uint32_t addr = (s->addr >> 10) * DF_PAGE_SIZE + (s->addr & 0x3ff);
            DPRINTF("we read(offset %X) %X\n", addr + s->cmd_len - 9, bytes[addr + s->cmd_len - 9]);

            return bytes[addr + s->cmd_len - 9];
        }

        return 0;
    case 0xD7:
        spi_flash_reset_state(s);
        DPRINTF("return id\n");
        return /*(1 << 2) | (1 << 3) | (1 << 5) | (1 << 7) AT45.*16*/
            (1 << 2) | (1 << 4) | (1 << 5) | (1 << 7);
    case 0x84://write to sram buf1        
        ++s->cmd_len;
        DPRINTF("cmd_len %u, value: %u\n", s->cmd_len, val);
        if (s->cmd_len > 2 && s->cmd_len < 5)
            s->addr |= (val & 0xFF) << ((1 - (s->cmd_len - 3)) * 8);
        else if (s->cmd_len < 5)
            DPRINTF("zero byte\n");
        else {
            DPRINTF("data idx %u\n", s->cmd_len - 5);
            s->buf1[s->cmd_len - 5] = val;
        }
        return 0;
    case 0x81://page erase
        ++s->cmd_len;
        if (s->cmd_len >= 2 && s->cmd_len <= 4) {
            s->addr |= (val & 0xFF) << ((4 - s->cmd_len) * 8);
        }
        if (s->cmd_len == 4) {
            uint8_t *bytes = s->storage;
            /*TODO: handle different page sizes*/
            uint32_t addr = (s->addr >> 10) * DF_PAGE_SIZE + (s->addr & 0x3ff);
            DPRINTF("we erase at %X\n", addr);
            memset(&bytes[addr], 0xFF, DF_PAGE_SIZE);
            if (bdrv_write(s->bs, (addr >> 9), bytes + (addr & ~(512 - 1)), (DF_PAGE_SIZE + 512-1) / 512) == -1)
                printf("%s: write error\n", __FUNCTION__);
        }
        return 0;
    case 0x88:
        ++s->cmd_len;
        if (s->cmd_len >= 2 && s->cmd_len <= 4) {
            s->addr |= (val & 0xFF) << ((4 - s->cmd_len) * 8);
        }
        if (s->cmd_len == 4) {
            uint8_t *bytes = s->storage;
            /*TODO: handle different page sizes*/
            uint32_t addr = (s->addr >> 10) * DF_PAGE_SIZE + (s->addr & 0x3ff);
            DPRINTF("program addr %X from buf1\n", addr);
            memcpy(&bytes[addr], s->buf1, DF_PAGE_SIZE);
            if (bdrv_write(s->bs, (addr >> 9), bytes + (addr & ~(512 - 1)), (DF_PAGE_SIZE + 512-1) / 512) == -1)
                printf("%s: write error\n", __FUNCTION__);
        }
        return 0;
    default:
        DPRINTF("Unknown cmd: %X\n", s->cmd);
        return 0;
    }
}

int spi_flash_register(BlockDriverState *bs, unsigned int len,
                       SPIControl *spi_control)
{
    SPIFlash *spi_flash;
    int ret = 0;

    spi_flash =  qemu_mallocz(sizeof(SPIFlash));
    spi_control->opaque = spi_flash;
    spi_control->txrx_callback = spi_flash_txrx;
    spi_control->set_chipselect = spi_flash_set_chipselect;

    spi_flash_reset_state(spi_flash);
    spi_flash->storage = qemu_malloc(len);
    spi_flash->buf1 = qemu_malloc(BUF1_SIZE);
    spi_flash->len = len;
    spi_flash->bs = bs;
    if (spi_flash->bs) {
        ret = bdrv_read(spi_flash->bs, 0, spi_flash->storage, len >> 9);
        if (ret < 0) {
            qemu_free(spi_flash->storage);
            qemu_free(spi_flash);
            goto out;
        }
    }
out:
    return ret;
}
