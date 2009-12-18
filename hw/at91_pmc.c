/*
 * AT91 Power Management Controller
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

#define PMC_SIZE        0x70

#define PMC_SCER        0x00 /* System Clock Enable Register */
#define PMC_SCDR        0x04 /* System Clock Disable Register */
#define PMC_SCSR        0x08 /* System Clock Status Register */
#define PMC_PCER        0x10 /* Peripheral Clock Enable Register */
#define PMC_PCDR        0x14 /* Peripheral Clock Disable Register */
#define PMC_PCSR        0x18 /* Peripheral Clock Status Register */
#define PMC_MOR         0x20 /* Main Oscillator Register */
#define PMC_MCFR        0x24 /* Main Clock Frequency Register */
#define PMC_PLLA        0x28 /* PLL A Register */
#define PMC_PLLB        0x2c /* PLL B Register */
#define PMC_MCKR        0x30 /* Master Clock Register */
#define PMC_PCKR        0x40 /* Programmable Clock Register */
#define PMC_IER         0x60 /* Interrupt Enable Register */
#define PMC_IDR         0x64 /* Interrupt Disable Register */
#define PMC_IMR         0x6c /* Interrupt Mask Register */
#define PMC_SR          0x68 /* Status Register */

#define SR_MOSCS        0x01
#define SR_LOCKA        0x02
#define SR_LOCKB        0x04
#define SR_MCKRDY       0x08
#define SR_PCK0RDY      0x100
#define SR_PCK1RDY      0x200
#define SR_PCK2RDY      0x400
#define SR_PCK3RDY      0x800

#define SO_FREQ         32768

int at91_master_clock_frequency = SO_FREQ;

typedef struct PMCState {
    SysBusDevice busdev;
    qemu_irq parent_irq;
    uint32_t scsr;
    uint32_t pcsr;
    uint32_t mor;
    uint32_t plla;
    uint32_t pllb;
    uint32_t mckr;
    uint32_t pckr[4];
    uint32_t sr;
    uint32_t imr;
    uint32_t mck_freq;
    uint32_t mo_freq;
} PMCState;

static void at91_pmc_update_irq(PMCState *s)
{
    qemu_set_irq(s->parent_irq, !!(s->sr & s->imr));
}

static void at91_update_master_clock(PMCState *s)
{
    int mck_freq = s->mo_freq;

    /* Clock selection */
    switch (s->mckr & 3) {
    case 0: /* Slow */
        mck_freq = SO_FREQ;
        break;
    case 1: /* Main */
        if (!(s->sr & SR_MOSCS))
            mck_freq = 0;
        break;
    case 2: /* PLL A */
        if ((s->plla & 0xff) != 0 &&
            (s->plla & 0x3ff80) != 0) {
            mck_freq /= s->plla & 0xff;
            mck_freq *= ((s->plla >> 16) & 0x7ff) + 1;
        } else {
            mck_freq = 0;
        }
        break;
    case 3: /* PLL B */
        if ((s->pllb & 0xff) != 0 &&
            (s->pllb & 0x3ff80) != 0) {
            mck_freq /= s->pllb & 0xff;
            mck_freq *= ((s->pllb >> 16) & 0x7ff) + 1;
        } else {
            mck_freq = 0;
        }
        break;
    }

    if (mck_freq != 0) {
        mck_freq /= 1 << ((s->mckr >> 2) & 7);
        mck_freq /= 1 << ((s->mckr >> 8) & 3);
        s->mck_freq = mck_freq;
        at91_master_clock_frequency = mck_freq;
        s->sr |= SR_MCKRDY;
    } else {
        s->sr &= ~SR_MCKRDY;
    }
}

static uint32_t at91_pmc_mem_read(void *opaque, target_phys_addr_t offset)
{
    PMCState *s = opaque;

    switch (offset) {
    case PMC_PLLA:
        return s->plla;
    case PMC_PLLB:
        return s->pllb;
    case PMC_SCSR:
        return s->scsr;
    case PMC_PCSR:
        return s->pcsr;
    case PMC_MOR:
        return s->mor;
    case PMC_MCFR:
        if (s->mor & 1)
            return (1 << 16) | (s->mo_freq / SO_FREQ / 16);
        return 0;
    case PMC_PCKR ... PMC_PCKR + 15:
        return s->pckr[(offset - PMC_PCKR) >> 2];
    case PMC_SR:
        return s->sr;
    case PMC_IMR:
        return s->imr;
    case PMC_MCKR:
        return s->mckr;
    default:
        return 0;
    }
}

static void at91_pmc_mem_write(void *opaque, target_phys_addr_t offset,
                uint32_t value)
{
    PMCState *s = opaque;

    switch (offset) {
    case PMC_SCER:
        s->scsr |= value & 0xf80;
        break;
    case PMC_SCDR:
        s->scsr &= ~(value & 0xf80);
        break;
    case PMC_PCER:
        s->pcsr |= value & ~3;
        break;
    case PMC_PCDR:
        s->pcsr &= ~(value & ~3);
        break;
    case PMC_MOR:
        /* Main Oscillator bypassing is not supported, so first two
           bits are ignored. Bits 8-15 specify the OSCOUNT, which is
           also currently ignored. */
        s->mor = value;
        s->sr |= SR_MOSCS;
        break;
    case PMC_PLLA:
        s->plla = value;
        /* OUTA, PLLACOUNT ignored for now */
        s->sr |= SR_LOCKA;
        break;
    case PMC_PLLB:
        s->pllb = value;
        /* OUTB, PLLBCOUNT ignored for now */
        s->sr |= SR_LOCKB;
        break;
    case PMC_MCKR:
        s->mckr = value;
        break;
    case PMC_PCKR ... PMC_PCKR + 15:
        s->pckr[(offset - PMC_PCKR) >> 2] = value;
        break;
    case PMC_IER:
        s->imr |= value;
        break;
    case PMC_IDR:
        s->imr &= ~value;
        break;
    default:
        return;
    }

    at91_update_master_clock(s);
    at91_pmc_update_irq(s);
}

static CPUReadMemoryFunc *at91_pmc_readfn[] = {
    at91_pmc_mem_read,
    at91_pmc_mem_read,
    at91_pmc_mem_read,
};

static CPUWriteMemoryFunc *at91_pmc_writefn[] = {
    at91_pmc_mem_write,
    at91_pmc_mem_write,
    at91_pmc_mem_write,
};

static void at91_pmc_save(QEMUFile *f, void *opaque)
{
    PMCState *s = opaque;
    int i;

    qemu_put_be32(f, s->scsr);
    qemu_put_be32(f, s->pcsr);
    qemu_put_be32(f, s->mor);
    qemu_put_be32(f, s->plla);
    qemu_put_be32(f, s->pllb);
    qemu_put_be32(f, s->mckr);
    for (i = 0; i < 4; i++) {
        qemu_put_be32(f, s->pckr[i]);
    }
    qemu_put_be32(f, s->sr);
    qemu_put_be32(f, s->imr);
    qemu_put_be32(f, s->mo_freq);
}

static int at91_pmc_load(QEMUFile *f, void *opaque, int version_id)
{
    PMCState *s = opaque;
    int i;

    if (version_id != 1)
        return -EINVAL;

    s->scsr = qemu_get_be32(f);
    s->pcsr = qemu_get_be32(f);
    s->mor = qemu_get_be32(f);
    s->plla = qemu_get_be32(f);
    s->pllb = qemu_get_be32(f);
    s->mckr = qemu_get_be32(f);
    for (i = 0; i < 4; i++) {
        s->pckr[i] = qemu_get_be32(f);
    }
    s->sr = qemu_get_be32(f);
    s->imr = qemu_get_be32(f);
    s->mo_freq = qemu_get_be32(f);

    at91_update_master_clock(s);

    return 0;
}

static void at91_pmc_reset(void *opaque)
{
    PMCState *s = opaque;

    s->scsr = 1;
    s->pcsr = 0;
    s->mor = 0;
    s->plla = s->pllb = 0x3f00;
    s->mckr = 0;
    s->pckr[0] = s->pckr[1] = s->pckr[2] = s->pckr[3] = 0;
    s->sr = 8;
    s->imr = 0;
    s->mck_freq = SO_FREQ;
}

static void at91_pmc_init(SysBusDevice *dev)
{
    PMCState *s = FROM_SYSBUS(typeof (*s), dev);
    int pmc_regs;

    sysbus_init_irq(dev, &s->parent_irq);

    pmc_regs = cpu_register_io_memory(at91_pmc_readfn, at91_pmc_writefn, s);
    sysbus_init_mmio(dev, PMC_SIZE, pmc_regs);

    at91_pmc_reset(s);
    qemu_register_reset(at91_pmc_reset, s);

    register_savevm("at91_pmc", -1, 1, at91_pmc_save, at91_pmc_load, s);
}

static SysBusDeviceInfo pmc_info = {
    .init = at91_pmc_init,
    .qdev.name = "at91,pmc",
    .qdev.size = sizeof(PMCState),
    .qdev.props = (Property[]) {
        {
            .name   = "mo_freq",
            .info   = &qdev_prop_uint32,
            .offset = offsetof(PMCState, mo_freq),
            .defval = (uint32_t[]) { 0 },
        },
       {/* end of list */}
    }
};

static void at91_pmc_register(void)
{
    sysbus_register_withprop(&pmc_info);
}

device_init(at91_pmc_register)
