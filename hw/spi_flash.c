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
} SPIFlash;

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
        s->cmd = val;
        ++s->cmd_len;
        return 0;
    case 0xE8:
        ++s->cmd_len;
        if (s->cmd_len >= 2 && s->cmd_len <= 4) {
            s->addr |= (val & 0xFF) << ((4 - s->cmd_len) * 8);
        } else if (s->cmd_len >= 5 && s->cmd_len < (5 + 4)) {
            /*ignore bytes*/
        } else {
            uint8_t *bytes = s->storage;
            /*TODO: handle different page sizes*/
            uint32_t addr = (s->addr >> 10) * 528 + (s->addr & 0x3ff);
            DPRINTF("we read %X\n", bytes[addr + s->cmd_len - 9]);

            return bytes[addr + s->cmd_len - 9];
        }

        return 0;
    case 0xD7:
        spi_flash_reset_state(s);
        DPRINTF("return id\n");
        return /*(1 << 2) | (1 << 3) | (1 << 5) | (1 << 7) AT45.*16*/
            (1 << 2) | (1 << 4) | (1 << 5) | (1 << 7);
    default:
        DPRINTF("Unknown cmd\n");
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
