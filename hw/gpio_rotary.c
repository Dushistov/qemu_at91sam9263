/*
 * GPIO Rotary Coder
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

typedef struct RotaryCoderState {
    SysBusDevice busdev;
    qemu_irq out[2];
    uint8_t state;
    uint8_t extension;
    uint32_t key_left;
    uint32_t key_right;
    uint32_t key_left_alt;
    uint32_t key_right_alt;
} RotaryCoderState;

static void rotary_update(RotaryCoderState *s, int direction)
{
    s->state += direction;
    s->state %= 4;

    qemu_set_irq(s->out[0], s->state == 1 || s->state == 2);
    qemu_set_irq(s->out[1], s->state == 2 || s->state == 3);
}

static void rotary_keyboard_event(void *opaque, int keycode)
{
    RotaryCoderState *s = opaque;

    if (keycode == 0xe0 && !s->extension) {
        s->extension = 0x80;
        return;
    }

    if (!(keycode & 0x80)) {
        keycode &= 0x7f;
        keycode |= s->extension;

		if (keycode == s->key_left || keycode == s->key_left_alt) {
            rotary_update(s, 3);
        } else if (keycode == s->key_right || keycode == s->key_right_alt) {
            rotary_update(s, 1);
        }
    }

    s->extension = 0;
}

static void rotary_save(QEMUFile *f, void *opaque)
{
    RotaryCoderState *s = opaque;

    qemu_put_byte(f, s->state);
    qemu_put_byte(f, s->extension);
}

static int rotary_load(QEMUFile *f, void *opaque, int version_id)
{
    RotaryCoderState *s = opaque;

    if (version_id != 1)
        return -EINVAL;

    s->state = qemu_get_byte(f);
    s->extension = qemu_get_byte(f);

    return 0;
}

/*
static void rotary_late_init(DeviceState *dev)
{
    RotaryCoderState *s = FROM_SYSBUS(RotaryCoderState, sysbus_from_qdev(dev));

    rotary_update(s);
}
*/

static void rotary_init(SysBusDevice *dev)
{
    RotaryCoderState *s = FROM_SYSBUS(RotaryCoderState, dev);

    qdev_init_gpio_out(&dev->qdev, s->out, 2);
    qemu_add_kbd_event_handler(rotary_keyboard_event, s);
    register_savevm("gpio_rotary", -1, 1, rotary_save, rotary_load, s);
}

static SysBusDeviceInfo rotary_info = {
    .init = rotary_init,
    /* .qdev.late_init = rotary_late_init, */
    .qdev.name  = "gpio,rotary",
    .qdev.size  = sizeof(RotaryCoderState),
    .qdev.props = (Property[]) {
        {
            .name   = "key-left",
            .info   = &qdev_prop_uint32,
            .offset = offsetof(RotaryCoderState, key_left),
            .defval = (uint32_t[]) { 0xcb },
        },
        {
            .name   = "key-right",
            .info   = &qdev_prop_uint32,
            .offset = offsetof(RotaryCoderState, key_right),
            .defval = (uint32_t[]) { 0xcd },
        },
        {
            .name   = "key-left-alt",
            .info   = &qdev_prop_uint32,
            .offset = offsetof(RotaryCoderState, key_left_alt),
            .defval = (uint32_t[]) { 0x4b },
        },
        {
            .name   = "key-right-alt",
            .info   = &qdev_prop_uint32,
            .offset = offsetof(RotaryCoderState, key_right_alt),
            .defval = (uint32_t[]) { 0x4d },
        },
        {/* end of list */}
    }
};

static void rotary_register(void)
{
    sysbus_register_withprop(&rotary_info);
}

device_init(rotary_register)
