/*
 * AT91 Debug Unit
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

/* TODO: Channel mode, ICE pins, FIFO?, overrun, retransmission */

#include "sysbus.h"
#include "qemu-char.h"
#include "at91.h"

/* #define DEBUG_DBGU */

#define DBGU_SIZE       0x200

#define DBGU_CR         0x00 /* Control Register */
#define DBGU_MR         0x04 /* Mode Register */
#define DBGU_IER        0x08 /* Interrupt Enable Register */
#define DBGU_IDR        0x0c /* Interrupt Disable Register */
#define DBGU_IMR        0x10 /* Interrupt Mask Register */
#define DBGU_SR         0x14 /* Channel Status Register */
#define DBGU_RHR        0x18 /* Receiver Holding Register */
#define DBGU_THR        0x1c /* Transmitter Holding Register */
#define DBGU_BRGR       0x20 /* Baud Rate Generator Register */
#define DBGU_CIDR       0x40 /* Chip ID Register */
#define DBGU_EXID       0x44 /* Chip ID Extension Register */
#define DBGU_FNR        0x48 /* Force NTRST Register */

#define SR_RXRDY        0x01 /* Reciever Ready */
#define SR_TXRDY        0x02 /* Transmitter Ready */
#define SR_ENDRX        0x08 /* End of Receive Transfer */
#define SR_ENDTX        0x10 /* End of Transmit */
#define SR_OVRE         0x20 /* Overrun */
#define SR_FRAME        0x40 /* Framing Error */
#define SR_PARE         0x80 /* Parity Error */
#define SR_TXEMPTY      0x200 /* Transmitter Empty */
#define SR_TXBUFE       0x800 /* Transmission Buffer Empty */
#define SR_RXBUFF       0x1000 /* Receiver Buffer Full */
#define SR_COMM_TX      0x40000000
#define SR_COMM_RX      0x80000000

#define CR_RXEN         0x10
#define CR_RXDIS        0x20
#define CR_TXEN         0x40
#define CR_TXDIS        0x80
#define CR_RSTSTA       0x100

#define DEFAULT_CHIPID  0x275b0940

typedef struct DBGUState {
    SysBusDevice busdev;
    CharDriverState *chr;
    qemu_irq irq;
    uint32_t chipid;
    uint32_t chipid_ext;

    uint8_t rx_enabled;
    uint8_t tx_enabled;
    uint32_t mr;
    uint32_t imr;
    uint32_t sr;
    uint8_t rhr;
    uint8_t thr;
    uint16_t brgr;
    uint32_t fnr;
} DBGUState;

static void at91_dbgu_update_irq(DBGUState *s)
{
    qemu_set_irq(s->irq, !!(s->sr & s->imr));
}

static void at91_dbgu_update_parameters(DBGUState *s)
{
    QEMUSerialSetParams ssp;

    if (s->brgr == 0)
        return;
    if (s->brgr == 1)
        ssp.speed = at91_master_clock_frequency / s->brgr;
    else
        ssp.speed = at91_master_clock_frequency / (16 * s->brgr);
    ssp.parity = 'O';
    ssp.data_bits = 8;
    ssp.stop_bits = 1;
    qemu_chr_ioctl(s->chr, CHR_IOCTL_SERIAL_SET_PARAMS, &ssp);
}

static void at91_dbgu_receive(void *opaque, const uint8_t *buf, int size)
{
    DBGUState *s = opaque;

    s->sr |= SR_RXRDY | SR_RXBUFF;
    s->rhr = (buf[0] & 0xff);
    at91_dbgu_update_irq(s);
}

static int at91_dbgu_can_receive(void *opaque)
{
    DBGUState *s = opaque;

    if (s->rx_enabled && !(s->sr & SR_RXRDY))
        return 1;
    return 0;
}

static void at91_dbgu_event(void *opaque, int event)
{
}

static uint32_t at91_dbgu_mem_read(void *opaque, target_phys_addr_t offset)
{
    DBGUState *s = opaque;
    uint32_t value;

    offset &= DBGU_SIZE - 1;
    switch (offset) {
    case DBGU_MR:
        return s->mr;
    case DBGU_IMR:
        return s->imr;
    case DBGU_SR:
        return s->sr;
    case DBGU_RHR:
        value = s->rhr;
        s->sr &= ~(SR_RXRDY | SR_RXBUFF);
        at91_dbgu_update_irq(s);
        return value;
    case DBGU_BRGR:
        return s->brgr;
    case DBGU_CIDR:
        return s->chipid;
    case DBGU_EXID:
        return s->chipid_ext;
    case DBGU_FNR:
        return s->fnr;
    default:
        return 0;
    }
}

static void at91_dbgu_mem_write(void *opaque, target_phys_addr_t offset,
                uint32_t value)
{
    DBGUState *s = opaque;
    unsigned char ch = value;

    offset &= DBGU_SIZE - 1;
    switch (offset) {
    case DBGU_CR:
        if (value & CR_RXDIS) {
            s->rx_enabled = 0;
        } else if (value & CR_RXEN) {
            s->rx_enabled = 1;
        }
        if (value & CR_TXDIS) {
            s->tx_enabled = 0;
        } else if (value & CR_TXEN) {
            s->tx_enabled = 1;
        }
        if (value & CR_RSTSTA) {
            s->sr &= ~(SR_PARE | SR_FRAME | SR_OVRE);
        }
        break;
    case DBGU_MR:
        s->mr = value;
        break;
    case DBGU_IER:
        s->imr |= value;
        break;
    case DBGU_IDR:
        s->imr &= ~value;
        break;
    case DBGU_THR:
        if (s->tx_enabled)
        {
            /* TODO: shift register, error checking */
            s->thr = value;
            qemu_chr_write(s->chr, &ch, 1);
            s->sr |= SR_TXRDY | SR_TXBUFE | SR_TXEMPTY;
        }
        break;
    case DBGU_BRGR:
        s->brgr = value;
        at91_dbgu_update_parameters(s);
        break;
    case DBGU_FNR:
        s->fnr = value;
        break;
    default:
        return;
    }

    at91_dbgu_update_irq(s);
}

#ifdef DEBUG_DBGU
static uint32_t at91_dbgu_mem_read_dbg(void *opaque, target_phys_addr_t offset)
{
    uint32_t value = at91_dbgu_mem_read(opaque, offset);
    printf("%s offset=%x val=%x\n", __func__, offset, value);
    return value;
}

static void at91_dbgu_mem_write_dbg(void *opaque, target_phys_addr_t offset,
                uint32_t value)
{
    printf("%s offset=%x val=%x\n", __func__, offset, value);
    at91_dbgu_mem_write(opaque, offset, value);
}

#define at91_dbgu_mem_read at91_dbgu_mem_read_dbg
#define at91_dbgu_mem_write at91_dbgu_mem_write_dbg
#endif

static CPUReadMemoryFunc *at91_dbgu_readfn[] = {
    at91_dbgu_mem_read,
    at91_dbgu_mem_read,
    at91_dbgu_mem_read,
};

static CPUWriteMemoryFunc *at91_dbgu_writefn[] = {
    at91_dbgu_mem_write,
    at91_dbgu_mem_write,
    at91_dbgu_mem_write,
};

static void at91_dbgu_save(QEMUFile *f, void *opaque)
{
    DBGUState *s = opaque;

    qemu_put_byte(f, s->rx_enabled);
    qemu_put_byte(f, s->tx_enabled);
    qemu_put_be32(f, s->mr);
    qemu_put_be32(f, s->imr);
    qemu_put_be32(f, s->sr);
    qemu_put_byte(f, s->rhr);
    qemu_put_byte(f, s->thr);
    qemu_put_be16(f, s->brgr);
    qemu_put_be32(f, s->fnr);
}

static int at91_dbgu_load(QEMUFile *f, void *opaque, int version_id)
{
    DBGUState *s = opaque;

    if (version_id != 1)
        return -EINVAL;

    s->rx_enabled = qemu_get_byte(f);
    s->tx_enabled = qemu_get_byte(f);
    s->mr = qemu_get_be32(f);
    s->imr = qemu_get_be32(f);
    s->sr = qemu_get_be32(f);
    s->rhr = qemu_get_byte(f);
    s->thr = qemu_get_byte(f);
    s->brgr = qemu_get_be16(f);
    s->fnr = qemu_get_be32(f);

    return 0;
}

static void at91_dbgu_reset(void *opaque)
{
    DBGUState *s = opaque;

    s->rx_enabled = 0;
    s->tx_enabled = 0;
    s->mr = 0;
    s->imr = 0;
    /* Transmitter begins ready and idle */
    s->sr = SR_TXRDY | SR_TXBUFE | SR_TXEMPTY;
    s->rhr = 0;
    s->thr = 0;
    s->brgr = 0;
    s->fnr = 0;
}

static void at91_dbgu_init(SysBusDevice *dev)
{
    DBGUState *s = FROM_SYSBUS(typeof (*s), dev);
    int ser_regs;

    sysbus_init_irq(dev, &s->irq);
    ser_regs = cpu_register_io_memory(at91_dbgu_readfn, at91_dbgu_writefn, s);
    sysbus_init_mmio(dev, DBGU_SIZE, ser_regs);
    s->chr = qdev_init_chardev(&dev->qdev);
    if (s->chr) {
        qemu_chr_add_handlers(s->chr, at91_dbgu_can_receive,
                              at91_dbgu_receive, at91_dbgu_event, s);
    }

    at91_dbgu_reset(s);
    qemu_register_reset(at91_dbgu_reset, s);

    register_savevm("at91_dbgu", -1, 1, at91_dbgu_save, at91_dbgu_load, s);
}

static SysBusDeviceInfo at91_dbgu_info = {
    .init = at91_dbgu_init,
    .qdev.name  = "at91,dbgu",
    .qdev.size  = sizeof(DBGUState),
    .qdev.props = (Property[]) {
        {
            .name   = "chipid",
            .info   = &qdev_prop_uint32,
            .offset = offsetof(DBGUState, chipid),
            .defval = (uint32_t[]) { DEFAULT_CHIPID },
        },
        {
            .name   = "chipid-ext",
            .info   = &qdev_prop_uint32,
            .offset = offsetof(DBGUState, chipid_ext)
        },
        {/* end of list */}
    }
};

static void at91_dbgu_register(void)
{
    sysbus_register_withprop(&at91_dbgu_info);
}

device_init(at91_dbgu_register)
