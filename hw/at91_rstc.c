/*
 * AT91 Reset Controller
 *
 * Copyright (c) 2009 Filip Navara
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "sysbus.h"

/* #define DEBUG_RSTC */

#define RSTC_SIZE        0x10

#define RSTC_CR          0x00 /* Control Register */
#define RSTC_SR          0x04 /* Status Register */
#define RSTC_MR          0x08 /* Mode Register */

#define WRITE_KEY        0xa5

#define CR_PROCRST       0x01 /* Processor Reset */
#define CR_PERRST        0x04 /* Peripheral Reset */
#define CR_EXTRST        0x08 /* External Reset */

#define SR_URSTS         0x01 /* User Reset Status */
#define SR_BODSTS        0x02 /* Brownout Detection Status */
#define SR_NRSTL         0x10000 /* NRST Level */

typedef struct RSTCState {
    SysBusDevice busdev;
    uint32_t sr;
    uint32_t mr;
} RSTCState;

static uint32_t at91_rstc_mem_read(void *opaque, target_phys_addr_t offset)
{
    RSTCState *s = opaque;

    offset &= RSTC_SIZE - 1;
    switch (offset) {
    case RSTC_SR:
        return s->sr;
    case RSTC_MR:
        return s->mr;
    default:
        return 0;
    }
}

static void at91_rstc_mem_write(void *opaque, target_phys_addr_t offset,
                uint32_t value)
{
    RSTCState *s = opaque;

    if ((value >> 24) != WRITE_KEY)
        return;

    offset &= RSTC_SIZE - 1;
    switch (offset) {
    case RSTC_CR:
        /* TODO */
        break;
    case RSTC_MR:
        s->mr = value;
        break;
    }
}

#ifdef DEBUG_RSTC
static uint32_t at91_rstc_mem_read_dbg(void *opaque, target_phys_addr_t offset)
{
    uint32_t value = at91_rstc_mem_read(opaque, offset);
    printf("%s offset=%x val=%x\n", __func__, offset, value);
    return value;
}

static void at91_rstc_mem_write_dbg(void *opaque, target_phys_addr_t offset,
                uint32_t value)
{
    printf("%s offset=%x val=%x\n", __func__, offset, value);
    at91_rstc_mem_write(opaque, offset, value);
}

#define at91_rstc_mem_read at91_rstc_mem_read_dbg
#define at91_rstc_mem_write at91_rstc_mem_write_dbg
#endif

static CPUReadMemoryFunc *at91_rstc_readfn[] = {
    at91_rstc_mem_read,
    at91_rstc_mem_read,
    at91_rstc_mem_read,
};

static CPUWriteMemoryFunc *at91_rstc_writefn[] = {
    at91_rstc_mem_write,
    at91_rstc_mem_write,
    at91_rstc_mem_write,
};

static void at91_rstc_save(QEMUFile *f, void *opaque)
{
    RSTCState *s = opaque;

    qemu_put_be32(f, s->sr);
    qemu_put_be32(f, s->mr);
}

static int at91_rstc_load(QEMUFile *f, void *opaque, int version_id)
{
    RSTCState *s = opaque;

    if (version_id != 1)
        return -EINVAL;

    s->sr = qemu_get_be32(f);
    s->mr = qemu_get_be32(f);

    return 0;
}

static void at91_rstc_reset(void *opaque)
{
    RSTCState *s = opaque;

    s->sr = SR_NRSTL;
    s->mr = 0;
}

static void at91_rstc_init(SysBusDevice *dev)
{
    RSTCState *s = FROM_SYSBUS(typeof (*s), dev);
    int rstc_regs;

    rstc_regs = cpu_register_io_memory(at91_rstc_readfn, at91_rstc_writefn, s);
    sysbus_init_mmio(dev, RSTC_SIZE, rstc_regs);

    at91_rstc_reset(s);
    qemu_register_reset(at91_rstc_reset, s);

    register_savevm("at91_rstc", -1, 1, at91_rstc_save, at91_rstc_load, s);
}

static void at91_rstc_register(void)
{
    sysbus_register_dev("at91,rstc", sizeof(RSTCState), at91_rstc_init);
}

device_init(at91_rstc_register)
