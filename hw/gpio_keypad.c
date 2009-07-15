/*
 * GPIO 4x4 Matrix Keyboard
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
#include "console.h"

typedef struct KeyPadState {
    SysBusDevice busdev;
    qemu_irq out[8];
    uint32_t in[8];
    uint32_t *keys;
    uint16_t keymask;
    uint16_t keymap[256];
    uint8_t extension;
} KeyPadState;

static void keypad_update(KeyPadState *s)
{
    int pin;
    int keymask;

    for (pin = 0; pin < 4; pin++) {
        if (s->keymask & (0xf << (pin * 4))) {
            keymask = s->keymask >> (pin * 4);
            qemu_set_irq(
                s->out[pin + 4],
                (s->in[0] && (keymask & 0x1)) ||
                (s->in[1] && (keymask & 0x2)) ||
                (s->in[2] && (keymask & 0x4)) ||
                (s->in[3] && (keymask & 0x8)));
        } else {
            qemu_set_irq(s->out[pin + 4], -1);
        }

        if (s->keymask & (0x1111 << pin)) {
            keymask = s->keymask >> pin;
            qemu_set_irq(
                s->out[pin],
                (s->in[4] && (keymask & 0x1)) ||
                (s->in[5] && (keymask & 0x10)) ||
                (s->in[6] && (keymask & 0x100)) ||
                (s->in[7] && (keymask & 0x1000)));
        } else {
            qemu_set_irq(s->out[pin], -1);
        }
    }
}

static void keypad_set_pin(void *opaque, int pin, int level)
{
    KeyPadState *s = opaque;

    s->in[pin] = level;
    keypad_update(s);
}

static void keypad_keyboard_event(void *opaque, int keycode)
{
    KeyPadState *s = opaque;
    int index;

    if (keycode == 0xe0 && !s->extension) {
        s->extension = 0x80;
        return;
    }

    index = (keycode & 0x7f) | s->extension;
    s->extension = 0;
    if (keycode & 0x80)
        s->keymask &= ~s->keymap[index];
    else if (keycode)
        s->keymask |= s->keymap[index];

    keypad_update(s);
}

static void keypad_save(QEMUFile *f, void *opaque)
{
    KeyPadState *s = opaque;
    int i;

    for (i = 0; i < 8; i++) {
        qemu_put_be32(f, s->in[i]);
    }
    qemu_put_be16(f, s->keymask);
    qemu_put_byte(f, s->extension);
}

static int keypad_load(QEMUFile *f, void *opaque, int version_id)
{
    KeyPadState *s = opaque;
    int i;

    if (version_id != 1)
        return -EINVAL;

    for (i = 0; i < 8; i++) {
        s->in[i] = qemu_get_be32(f);
    }
    s->keymask = qemu_get_be16(f);
    s->extension = qemu_get_byte(f);

    return 0;
}


/*
static void keypad_late_init(DeviceState *dev)
{
    KeyPadState *s = FROM_SYSBUS(KeyPadState, sysbus_from_qdev(dev));

    keypad_update(s);
}
*/

static void keypad_init(SysBusDevice *dev)
{
    KeyPadState *s = FROM_SYSBUS(KeyPadState, dev);
    int i;

    for (i = 0; i < 16; i++) {
        s->keymap[s->keys[i] & 0xff] = 1 << i;
    }

    qdev_init_gpio_in(&dev->qdev, keypad_set_pin, 8);
    qdev_init_gpio_out(&dev->qdev, s->out, 8);
    qemu_add_kbd_event_handler(keypad_keyboard_event, s);
    register_savevm("gpio_keypad", -1, 1, keypad_save, keypad_load, s);
}

static SysBusDeviceInfo keypad_info = {
    .init = keypad_init,
    /* .qdev.late_init = keypad_late_init, */
    .qdev.name  = "gpio,keypad",
    .qdev.size  = sizeof(KeyPadState),
    .qdev.props = (Property[]) {
        {
            .name   = "keys",
            .info   = &qdev_prop_ptr, /* FIXME: Make it array! */
            .offset = offsetof(KeyPadState, keys),
        },
        {/* end of list */}
    }
};

static void keypad_register(void)
{
    sysbus_register_withprop(&keypad_info);
}

device_init(keypad_register)
