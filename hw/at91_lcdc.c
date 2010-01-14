/*
 * AT91 SAM9 LCD Controller (LCDC)
 * Written by Evgeniy Dushistov
 * This code is licenced under the GPL.
 */
#include "sysbus.h"
#include "console.h"
#include "pixel_ops.h"

#define LCDC_SIZE 0x100000

#define LCDC_DMABADDR1 0x0
#define LCDC_DMAFRMCFG 0x18
#define LCDC_DMACON    0x1C
#define LCDC_LCDCON1   0x800
#define LCDC_LCDCON2   0x804
#define LCDC_LCDFRMCFG 0x810
#define LCDC_PWRCON    0x83C

#define DMACON_DMAEN 1

typedef struct LCDCState {
    SysBusDevice busdev;
    qemu_irq irq;
    DisplayState *ds;
    uint32_t dmacon;
    uint32_t pwrcon;
    uint32_t dmafrmcfg;
    uint32_t lcdcon1;
    uint32_t lcdcon2;
    uint32_t lcdfrmcfg;
    uint32_t dmabaddr1;
} LCDCState;

#define AT91_LCDC_DEBUG
#ifdef AT91_LCDC_DEBUG
#define DPRINTF(fmt, ...)                           \
    do {                                            \
        printf("AT91LCD: " fmt , ## __VA_ARGS__);    \
    } while (0)
#else
#define DPRINTF(fmt, ...) do { } while (0)
#endif


static uint32_t at91_lcdc_mem_read(void *opaque, target_phys_addr_t offset)
{
    LCDCState *s = opaque;

    offset &= LCDC_SIZE - 1;
    DPRINTF("read from %X\n", offset);
    switch (offset) {
    case LCDC_PWRCON:
        return s->pwrcon;
    case LCDC_DMACON:
        return s->dmacon;
    case LCDC_DMAFRMCFG:
        return s->dmafrmcfg;
    case LCDC_LCDCON1:
        return s->lcdcon1;
    case LCDC_LCDCON2:
        return s->lcdcon2;
    case LCDC_LCDFRMCFG:
        return s->lcdfrmcfg;
    default:
        DPRINTF("unsup. read\n");
        return 0;
    }
}

static void at91_lcdc_mem_write(void *opaque, target_phys_addr_t offset,
                uint32_t value)
{
    LCDCState *s = opaque;
    offset &= LCDC_SIZE - 1;
    DPRINTF("write to %X: %X\n", offset, value);

    switch (offset) {
    case LCDC_DMABADDR1:
        s->dmabaddr1 = value;
        break;
    case LCDC_PWRCON:
        s->pwrcon = value;
        break;
    case LCDC_DMACON:
        s->dmacon = value;
        break;
    case LCDC_DMAFRMCFG:
        s->dmafrmcfg = value;
        break;
    case LCDC_LCDCON1:
        s->lcdcon1 = value;
        break;
    case LCDC_LCDCON2:
        DPRINTF("pixel size %u\n", (value >> 5) & 7);
        s->lcdcon2 = value;
        break;
     case LCDC_LCDFRMCFG:
        DPRINTF("lineval %u, linsize %u\n", value & 0x7FF, (value >> 21) & 0x7FF);
        qemu_console_resize(s->ds, ((value >> 21) & 0x7FF) + 1, (value & 0x7FF) + 1);
        s->lcdfrmcfg = value;
        break;
    default:
        DPRINTF("unsup. write\n");
    }
}

static CPUReadMemoryFunc *at91_lcdc_readfn[] = {
    at91_lcdc_mem_read,
    at91_lcdc_mem_read,
    at91_lcdc_mem_read,
};

static CPUWriteMemoryFunc *at91_lcdc_writefn[] = {
    at91_lcdc_mem_write,
    at91_lcdc_mem_write,
    at91_lcdc_mem_write,
};

static void at91_lcdc_reset(void *opaque)
{
    LCDCState *s = opaque;
    s->pwrcon = 0x0000000e;
    s->dmacon = 0;
    s->dmafrmcfg = 0;
    s->lcdcon1 = 0x00002000;
    s->lcdfrmcfg = 0;
}

struct pixel8 {
    unsigned b : 2;
    unsigned g : 3;
    unsigned r : 3;
};

union pixel8u {
    struct pixel8 p;
    uint8_t val;
};

struct pixel16 {
    unsigned b : 5;
    unsigned g : 5;
    unsigned r : 5;
};

union pixel16u {
    struct pixel16 p;
    uint8_t val[0];
};


static void at91_lcdc_update_display(void *opaque)
{
    LCDCState *s = opaque;
    uint32_t color;
    int x, y;
    uint8_t *d;
    int width = ((s->lcdfrmcfg >> 21) & 0x7FF) + 1;
    int height = (s->lcdfrmcfg & 0x7FF) + 1;
    int q_bpp = (ds_get_bits_per_pixel(s->ds) + 7) >> 3;
    int bpp;
    int bpp_idx = (s->lcdcon2 >> 5) & 7;
    union pixel8u tmp8;
    union pixel16u tmp16;
    unsigned int r, g, b;

    DPRINTF("update begin\n");
    if (!(s->dmacon & DMACON_DMAEN))
        return;
    DPRINTF("update continue\n");
    switch (bpp_idx) {
    case 0 ... 4:
        bpp = 1 << bpp_idx;
        break;
    case 5 ... 6:
        bpp = 24;
        break;
    default:
        fprintf(stderr, "Unknown pixel size\n");
        return;//reserved value, unknown pixel size
    }
    /*TODO: fix this restriction*/
    if (bpp != 8 && bpp != 16) {
        fprintf(stderr, "Unsupported pixel size: %d\n", bpp);
        return;
    }
    //int once = 0;
    for (y = 0; y < height; ++y) {        
        for (x = 0; x < width; ++x) {
            if (bpp == 8) {
                cpu_physical_memory_read(s->dmabaddr1 + width * y + x, &tmp8.val, 1);
                r = tmp8.p.r;
                g = tmp8.p.g;
                b = tmp8.p.b;
            } else {
                cpu_physical_memory_read(s->dmabaddr1 + width * y + x, &tmp16.val[0], 2);
                r = tmp16.p.r;
                g = tmp16.p.g;
                b = tmp16.p.b;
            }
#if 0
            if (tmp.val != 0 && once == 0) {
                once = 1;
                DPRINTF("not null %X, %X, %X, bpp %d\n", (unsigned)r << 5, (unsigned)g << 5, (unsigned)b << 6,
                        ds_get_bits_per_pixel(s->ds));
            }
#endif
            switch (ds_get_bits_per_pixel(s->ds)) {
            case 8:
                color = rgb_to_pixel8(r, g, b);
                break;
            case 15:
                color = rgb_to_pixel15(r, g, b);
                break;
            case 16:
                color = rgb_to_pixel16(r, g, b);
                break;
            case 24:
                color = rgb_to_pixel24(r, g, b);
                break;
            case 32:
                if (bpp == 8)
                    color = rgb_to_pixel32((unsigned)r << 5, (unsigned)g << 5, (unsigned)b << 6);
                else
                    color = rgb_to_pixel32((unsigned)r << 3, (unsigned)g << 3, (unsigned)b << 3);
                break;
            default:
                return;
            }

            d = ds_get_data(s->ds) + ds_get_linesize(s->ds) * y + q_bpp * x;
            *d = color;
        }
    }

    dpy_update(s->ds, 0, 0, width, height);
}

static void at91_lcdc_init(SysBusDevice *dev)
{
    LCDCState *s = FROM_SYSBUS(typeof(*s), dev);
    int lcdc_regs;

    sysbus_init_irq(dev, &s->irq);
    lcdc_regs = cpu_register_io_memory(at91_lcdc_readfn,
                                      at91_lcdc_writefn, s);
    sysbus_init_mmio(dev, LCDC_SIZE, lcdc_regs);    
    qemu_register_reset(at91_lcdc_reset, s);
    s->ds = graphic_console_init(at91_lcdc_update_display, NULL, NULL, NULL, s);
}

static void at91_lcdc_register(void)
{
    sysbus_register_dev("at91,lcdc", sizeof(LCDCState), at91_lcdc_init);
}

device_init(at91_lcdc_register)
