/*
 * AT91 Periodic Interval Timer
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
#include "qemu-timer.h"
#include "at91.h"

#define PIT_SIZE        0x10

#define PIT_MR          0x00 /* Mode Register */
#define PIT_SR          0x04 /* Status Register */
#define PIT_PIVR        0x08 /* Periodic Interval Value Register */
#define PIT_PIIR        0x0c /* Periodic Interval Image Register */

#define PIT_LIMIT(s) \
    (((s)->mr & 0xfffff) + 1)

typedef struct PITState {
    SysBusDevice busdev;
    qemu_irq irq;
    ptimer_state *timer;
    uint32_t mr;
    uint32_t sr;
    uint32_t picnt;
} PITState;

static void at91_pit_tick(void *opaque)
{
    PITState *s = opaque;

    s->sr |= 1;
    s->picnt++;
    if (s->mr & 0x2000000) {
        qemu_set_irq(s->irq, 1);
    }
}

static uint32_t at91_pit_mem_read(void *opaque, target_phys_addr_t offset)
{
    PITState *s = opaque;
    uint32_t picnt = s->picnt;

    offset &= 0xf;
    switch (offset) {
    case PIT_MR:
        return s->mr;
    case PIT_SR:
        return s->sr;
    case PIT_PIVR:
        s->sr = 0;
        s->picnt = 0;
        qemu_set_irq(s->irq, 0);
        /* Fall-through */
    case PIT_PIIR:
        return
            ((PIT_LIMIT(s) - ptimer_get_count(s->timer)) & 0xfffff) |
            (picnt << 20);

    default:
        return 0;
    }
}

static void at91_pit_mem_write(void *opaque, target_phys_addr_t offset,
                uint32_t value)
{
    PITState *s = opaque;

    offset &= 0xf;
    if (offset == PIT_MR) {
        s->mr = value;
        if (value & 0x1000000) {
            ptimer_set_freq(s->timer, at91_master_clock_frequency / 16);
            ptimer_set_limit(s->timer, PIT_LIMIT(s), 1);
            ptimer_run(s->timer, 0);
        } else {
            ptimer_stop(s->timer);
            /*
              "After the PIT Enable bit is reset (PITEN= 0), the CPIV goes on counting until
              the PIV value is reached, and is then reset"
              Do not wait, set CPIV to right value now.
            */
            ptimer_set_count(s->timer, PIT_LIMIT(s));
        }
    }
}

static CPUReadMemoryFunc *at91_pit_readfn[] = {
    at91_pit_mem_read,
    at91_pit_mem_read,
    at91_pit_mem_read,
};

static CPUWriteMemoryFunc *at91_pit_writefn[] = {
    at91_pit_mem_write,
    at91_pit_mem_write,
    at91_pit_mem_write,
};

static void at91_pit_save(QEMUFile *f, void *opaque)
{
    PITState *s = opaque;

    qemu_put_be32(f, s->mr);
    qemu_put_be32(f, s->sr);
    qemu_put_be32(f, s->picnt);
    qemu_put_ptimer(f, s->timer);
}

static int at91_pit_load(QEMUFile *f, void *opaque, int version_id)
{
    PITState *s = opaque;

    if (version_id != 1)
        return -EINVAL;

    s->mr = qemu_get_be32(f);
    s->sr = qemu_get_be32(f);
    s->picnt = qemu_get_be32(f);
    qemu_get_ptimer(f, s->timer);

    return 0;
}

static void at91_pit_reset(void *opaque)
{
    PITState *s = opaque;

    s->mr = 0xfffff;
    s->sr = 0;
    s->picnt = 0;
    ptimer_stop(s->timer);
}

static void at91_pit_init(SysBusDevice *dev)
{
    PITState *s = FROM_SYSBUS(typeof (*s), dev);
    QEMUBH *pit_bh;
    int pit_regs;

    pit_bh = qemu_bh_new(at91_pit_tick, s);
    s->timer = ptimer_init(pit_bh);

    sysbus_init_irq(dev, &s->irq);
    pit_regs = cpu_register_io_memory(at91_pit_readfn, at91_pit_writefn, s);
    sysbus_init_mmio(dev, PIT_SIZE, pit_regs);

    at91_pit_reset(s);
    qemu_register_reset(at91_pit_reset, s);

    register_savevm("at91_pit", -1, 1, at91_pit_save, at91_pit_load, s);
}

static void at91_pit_register(void)
{
    sysbus_register_dev("at91,pit", sizeof(PITState), at91_pit_init);
}

device_init(at91_pit_register)
