/*
 * AT91 PES Development Board
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
#include "boards.h"
#include "arm-misc.h"
#include "sysemu.h"
#include "net.h"

static void at91pes_init(ram_addr_t ram_size,
                      const char *boot_device,
                      const char *kernel_filename, const char *kernel_cmdline,
                      const char *initrd_filename, const char *cpu_model)
{
    struct arm_boot_info at91pes_binfo;
    CPUState *env;
    qemu_irq *cpu_pic;
    qemu_irq pic[32];
    qemu_irq pic1[32];
    ram_addr_t ram_addr;
    DeviceState *dev;
    DeviceState *pioa;
    DeviceState *piob;
    DeviceState *pmc;
    DeviceState *pit;
    uint32_t keys[] = {
            2 /* 1 */, 3 /* 2 */, 4 /* 3 */, 30 /* A */,
            5 /* 4 */, 6 /* 5 */, 7 /* 6 */, 48 /* B */,
            8 /* 7 */, 9 /* 8 */, 10 /* 9 */, 46 /* C */,
            42 /* LShift */, 11 /* 0 */, 12 /* - */, 32 /* D */
        };
    int i;

    if (!cpu_model)
        cpu_model = "arm7tdmi";
    env = cpu_init(cpu_model);
    if (!env) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }

    /* RAM at address zero. */
    ram_addr = qemu_ram_alloc(ram_size);
    cpu_register_physical_memory(0, 0x100000, ram_addr | IO_MEM_RAM);
    cpu_register_physical_memory(0x200000, ram_size, ram_addr | IO_MEM_RAM);

    cpu_pic = arm_pic_init_cpu(env);
    dev = sysbus_create_varargs("at91,aic", 0xFFFFF000,
                                cpu_pic[ARM_PIC_CPU_IRQ],
                                cpu_pic[ARM_PIC_CPU_FIQ],
                                NULL);
    for (i = 0; i < 32; i++) {
        pic[i] = qdev_get_gpio_in(dev, i);
    }

    dev = sysbus_create_simple("at91,intor", -1, pic[1]);
    for (i = 0; i < 32; i++) {
        pic1[i] = qdev_get_gpio_in(dev, i);
    }

    sysbus_create_simple("at91,dbgu", 0xFFFFF200, pic1[0]);
    pmc = sysbus_create_simple("at91,pmc", 0xFFFFFC00, pic1[1]);
    qdev_prop_set_uint32(pmc, "mo_freq", 9216000);
    sysbus_create_varargs("at91,rstc", 0xFFFFFD00, NULL);
    pioa = sysbus_create_simple("at91,pio", 0xFFFFF400, pic[2]);
    piob = sysbus_create_simple("at91,pio", 0xFFFFF600, pic[3]);
    sysbus_create_simple("at91,rtt", 0xFFFFFD20, pic1[2]);
    pit = sysbus_create_simple("at91,pit", 0xFFFFFD30, pic1[3]);
    sysbus_create_varargs("at91,tc", 0xFFFA0000, pic[12], pic[13], pic[14], NULL);

    qemu_check_nic_model(&nd_table[0], "at91");
    dev = qdev_create(NULL, "at91,emac");
    dev->nd = &nd_table[0];
    qdev_init(dev);
    sysbus_mmio_map(sysbus_from_qdev(dev), 0, 0xFFFDC000);
    sysbus_connect_irq(sysbus_from_qdev(dev), 0, pic[16]);

    dev = qdev_create(NULL, "gpio,hd44780");
    qdev_init(dev);
    for (i = 0; i < 3; i++) {
        qdev_connect_gpio_out(pioa, 15 - i, qdev_get_gpio_in(dev, 8 + i));
    }
    qdev_connect_gpio_out(piob, 28, qdev_get_gpio_in(dev, 11));
    for (i = 0; i < 4; i++) {
        qdev_connect_gpio_out(pioa, 5 + i, qdev_get_gpio_in(dev, 4 + i));
        qdev_connect_gpio_out(dev, 4 + i, qdev_get_gpio_in(pioa, 5 + i));
    }

    dev = qdev_create(NULL, "gpio,keypad");
    qdev_prop_set_ptr(dev, "keys", keys);
    qdev_init(dev);
    for (i = 0; i < 4; i++) {
        qdev_connect_gpio_out(pioa, 21 + i, qdev_get_gpio_in(dev, 4 + i));
        qdev_connect_gpio_out(dev, 4 + i, qdev_get_gpio_in(pioa, 21 + i));
    }
    for (i = 0; i < 2; i++) {
        qdev_connect_gpio_out(pioa, 26 - i, qdev_get_gpio_in(dev, 2 + i));
        qdev_connect_gpio_out(dev, 2 + i, qdev_get_gpio_in(pioa, 26 - i));
        qdev_connect_gpio_out(pioa, 30 - i, qdev_get_gpio_in(dev, i));
        qdev_connect_gpio_out(dev, i, qdev_get_gpio_in(pioa, 30 - i));
    }

    dev = qdev_create(NULL, "gpio,rotary");
    qdev_prop_set_uint32(dev, "key-left", 0xcb);
    qdev_prop_set_uint32(dev, "key-right", 0xcd);
    qdev_init(dev);
    qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in(piob, 22));
    qdev_connect_gpio_out(dev, 1, qdev_get_gpio_in(piob, 23));

    at91pes_binfo.ram_size = ram_size;
    at91pes_binfo.kernel_filename = kernel_filename;
    at91pes_binfo.kernel_cmdline = kernel_cmdline;
    at91pes_binfo.initrd_filename = initrd_filename;
    at91pes_binfo.board_id = 0;
    arm_load_kernel(env, &at91pes_binfo);
}

static QEMUMachine at91pes_machine = {
    .name = "at91pes",
    .desc = "Atmel AT91SAM7X PES Development Board",
    .init = at91pes_init,
};

static void at91pes_machine_init(void)
{
    qemu_register_machine(&at91pes_machine);
}

machine_init(at91pes_machine_init);
