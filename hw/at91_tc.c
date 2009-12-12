/*
 * AT91 Timer Counter
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

#define TC_SIZE         0x4000

#define TC_CCR          0x00 /* Channel Control Register */
#define TC_CMR          0x04 /* Channel Mode Register */
#define TC_CV           0x10 /* Counter Value Register */
#define TC_RA           0x14 /* Register A */
#define TC_RB           0x18 /* Register B */
#define TC_RC           0x1c /* Register C */
#define TC_SR           0x20 /* Status Register */
#define TC_IER          0x24 /* Interrupt Enable Register */
#define TC_IDR          0x28 /* Interrupt Disable Register */
#define TC_IMR          0x2c /* Interrupt Mask Register */
#define TC_BCR          0xc0 /* Block Control Register */
#define TC_BMR          0xc4 /* Block Mode Register */

#define MR_CLKI         0x08 /* Clock Invert */
#define MR_CPCSTOP      0x40 /* Counter Clock Stopped with RC Compare */
#define MR_CPCDIS       0x80 /* Counter Clock Disable with RC Compare */
#define MR_RCTRIG       0x4000
#define MR_WAVE         0x8000

#define SR_CPAS         0x04
#define SR_CPBS         0x08
#define SR_CPCS         0x10

typedef struct TCChannelState {
    qemu_irq irq;
    ptimer_state *timer;
    uint32_t mr;
    uint16_t cv;
    uint16_t ra;
    uint16_t rb;
    uint16_t rc;
    uint32_t sr;
    uint32_t imr;
} TCChannelState;

typedef struct TCState {
    SysBusDevice busdev;
    TCChannelState channels[3];
} TCState;

static void at91_tc_tick(void *opaque)
{
    TCChannelState *s = opaque;

    if (s->mr & MR_WAVE) {
        s->cv++;
        /* TODO: Overflow check */
        if (s->cv == s->ra) {
            s->sr |= SR_CPAS;
        }
        if (s->cv == s->rb) {
            s->sr |= SR_CPBS;
        }
        if (s->cv == s->rc) {
            s->sr |= SR_CPCS;
            if (s->mr & MR_RCTRIG) {
                s->cv = 0;
            }
        }
    } else {
        s->cv = s->rc;
        s->sr |= SR_CPCS;
        if (s->mr & MR_RCTRIG) {
            s->cv = 0;
        }
    }
    if (s->sr & s->imr) {
        qemu_set_irq(s->irq, 1);
    }
}

static uint32_t at91_tc_channel_read(TCChannelState *s,
                    target_phys_addr_t offset)
{
    uint32_t sr;

    switch (offset) {
    case TC_CMR:
        return s->mr;
    case TC_CV:
        return s->cv;
    case TC_RA:
        return s->ra;
    case TC_RB:
        return s->rb;
    case TC_RC:
        return s->rc;
    case TC_SR:
        sr = s->sr;
        s->sr = 0;
        qemu_set_irq(s->irq, 0);
        return sr;
    case TC_IMR:
        return s->imr;
    default:
        return 0;
    }
}

static int at91_tc_get_freq(uint32_t cmr)
{
    int freq;

    switch (cmr & 7) {
    case 0: freq = at91_master_clock_frequency / 2; break;
    case 1: freq = at91_master_clock_frequency / 8; break;
    case 2: freq = at91_master_clock_frequency / 32; break;
    case 3: freq = at91_master_clock_frequency / 128; break;
    case 4: freq = at91_master_clock_frequency / 1024; break;
    default: /* TODO: External clocks */
        freq = at91_master_clock_frequency / 16;
        break;
    }
    return freq;
}

static void at91_tc_channel_write(TCChannelState *s,
                target_phys_addr_t offset, uint32_t value)
{
    int freq;

    switch (offset) {
    case TC_CCR:
        if ((value & 3) == 1) {
            if (!(s->mr & MR_WAVE)) {
                if (s->imr == 0 && s->rc != 0) {
                    freq = at91_tc_get_freq(s->mr);
                    freq /= s->rc;
                    if (freq > 1000) {
                        //just busy loop too wait something, with resolution more then 1ms
                        s->cv = s->rc;
                        s->sr |= SR_CPCS;
                        if (s->mr & MR_RCTRIG) {
                            s->cv = 0;
                        }
                        break;
                    }
                }
                //tick only once, this should speedup system
                ptimer_set_limit(s->timer, s->rc, 1);
            }

            s->cv = 0;
            ptimer_run(s->timer, 0);
        } else if (value & 2) {
            ptimer_stop(s->timer);
            ptimer_set_limit(s->timer, 1, 1);
        }
        break;
    case TC_CMR:
        freq = at91_tc_get_freq(value);

        ptimer_set_freq(s->timer, freq);

        s->mr = value;
        break;
    case TC_RA:
        s->ra = value;
        break;
    case TC_RB:
        s->rb = value;
        break;
    case TC_RC:
        s->rc = value;
        break;
    case TC_IER:
        s->imr |= value;
        break;
    case TC_IDR:
        s->imr &= ~value;
        break;
    }
}

static uint32_t at91_tc_mem_read(void *opaque, target_phys_addr_t offset)
{
    TCState *s = opaque;

    offset &= TC_SIZE - 1;
    switch (offset) {
    case TC_BMR:
        return 0; /* TODO */
    case 0 ... 0x3f:
        return at91_tc_channel_read(&s->channels[0], offset);
    case 0x40 ... 0x7f:
        return at91_tc_channel_read(&s->channels[1], offset - 0x40);
    case 0x80 ... 0xbf:
        return at91_tc_channel_read(&s->channels[2], offset - 0x80);
    default:
        return 0;
    }
}

static void at91_tc_mem_write(void *opaque, target_phys_addr_t offset,
                uint32_t value)
{
    TCState *s = opaque;

    offset &= TC_SIZE - 1;

    switch (offset) {
    case TC_BCR:
        return; /* TODO */
    case TC_BMR:
        return; /* TODO */
    case 0 ... 0x3f:
        at91_tc_channel_write(&s->channels[0], offset, value);
        break;
    case 0x40 ... 0x7f:
        at91_tc_channel_write(&s->channels[1], offset - 0x40, value);
        break;
    case 0x80 ... 0xbf:
        at91_tc_channel_write(&s->channels[2], offset - 0x80, value);
        break;
    }
}

static CPUReadMemoryFunc *at91_tc_readfn[] = {
    at91_tc_mem_read,
    at91_tc_mem_read,
    at91_tc_mem_read,
};

static CPUWriteMemoryFunc *at91_tc_writefn[] = {
    at91_tc_mem_write,
    at91_tc_mem_write,
    at91_tc_mem_write,
};

static void at91_tc_save(QEMUFile *f, void *opaque)
{
    TCState *s = opaque;
    int channel;

    for (channel = 0; channel < 3; channel++) {
        qemu_put_ptimer(f, s->channels[channel].timer);
        qemu_put_be32(f, s->channels[channel].mr);
        qemu_put_be16(f, s->channels[channel].cv);
        qemu_put_be16(f, s->channels[channel].ra);
        qemu_put_be16(f, s->channels[channel].rb);
        qemu_put_be16(f, s->channels[channel].rc);
        qemu_put_be32(f, s->channels[channel].sr);
        qemu_put_be32(f, s->channels[channel].imr);
    }
}

static int at91_tc_load(QEMUFile *f, void *opaque, int version_id)
{
    TCState *s = opaque;
    int channel;

    if (version_id != 1)
        return -EINVAL;

    for (channel = 0; channel < 3; channel++) {
        qemu_get_ptimer(f, s->channels[channel].timer);
        s->channels[channel].mr = qemu_get_be32(f);
        s->channels[channel].cv = qemu_get_be16(f);
        s->channels[channel].ra = qemu_get_be16(f);
        s->channels[channel].rb = qemu_get_be16(f);
        s->channels[channel].rc = qemu_get_be16(f);
        s->channels[channel].sr = qemu_get_be32(f);
        s->channels[channel].imr = qemu_get_be32(f);
    }

    return 0;
}

static void at91_tc_reset(void *opaque)
{
    TCState *s = opaque;
    int channel;

    for (channel = 0; channel < 3; channel++) {
        ptimer_stop(s->channels[channel].timer);
        s->channels[channel].mr = 0;
        s->channels[channel].cv = 0;
        s->channels[channel].ra = 0;
        s->channels[channel].rb = 0;
        s->channels[channel].rc = 0;
        s->channels[channel].sr = 0;
        s->channels[channel].imr = 0;
    }
}

static void at91_tc_init(SysBusDevice *dev)
{
    TCState *s = FROM_SYSBUS(typeof (*s), dev);
    QEMUBH *bh;
    int tc_regs;
    int channel;

    for (channel = 0; channel < 3; channel++) {
        bh = qemu_bh_new(at91_tc_tick, &s->channels[channel]);
        s->channels[channel].timer = ptimer_init(bh);
        ptimer_set_limit(s->channels[channel].timer, 1, 1);
        sysbus_init_irq(dev, &s->channels[channel].irq);
    }

    tc_regs = cpu_register_io_memory(at91_tc_readfn,
                  at91_tc_writefn, s);
    sysbus_init_mmio(dev, TC_SIZE, tc_regs);

    qemu_register_reset(at91_tc_reset, s);

    register_savevm("at91_tc", -1, 1, at91_tc_save, at91_tc_load, s);
}

static void at91_tc_register(void)
{
    sysbus_register_dev("at91,tc", sizeof(TCState), at91_tc_init);
}

device_init(at91_tc_register)
