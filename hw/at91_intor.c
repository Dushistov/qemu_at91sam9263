/*
 * AT91 Interrupt Logic OR
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

typedef struct IntOrState {
    SysBusDevice busdev;
    qemu_irq parent_irq;
    uint32_t sources;
} IntOrState;

static void at91_intor_set_irq(void *opaque, int irq, int level)
{
    IntOrState *s = opaque;

    if (level) {
        s->sources |= 1 << irq;
    } else {
        s->sources &= ~(1 << irq);
    }
    qemu_set_irq(s->parent_irq, !!s->sources);
}

static void at91_intor_save(QEMUFile *f, void *opaque)
{
    IntOrState *s = opaque;

    qemu_put_be32(f, s->sources);
}

static int at91_intor_load(QEMUFile *f, void *opaque, int version_id)
{
    IntOrState *s = opaque;

    if (version_id != 1)
        return -EINVAL;

    s->sources = qemu_get_be32(f);
    return 0;
}

static void at91_intor_reset(void *opaque)
{
    IntOrState *s = opaque;
    
    s->sources = 0;
}

static void at91_intor_init(SysBusDevice *dev)
{
    IntOrState *s = FROM_SYSBUS(typeof (*s), dev);

    qdev_init_gpio_in(&dev->qdev, at91_intor_set_irq, 32);
    sysbus_init_irq(dev, &s->parent_irq);

    qemu_register_reset(at91_intor_reset, s);

    register_savevm("at91_intor", -1, 1, at91_intor_save, at91_intor_load, s);
}

static void at91_intor_register(void)
{
    sysbus_register_dev("at91,intor", sizeof(IntOrState), at91_intor_init);
}

device_init(at91_intor_register)
