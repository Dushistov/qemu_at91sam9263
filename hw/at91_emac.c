/*
 * AT91 Ethernet MAC
 *
 * Copyright (c) 2007, 2009 Filip Navara
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

/*
 * TODO:
 * PHY read/write (MII/RMII), statistics, VLAN, hash addressing, jumbo frame
 * sending, ...
 */

#include "sysbus.h"
#include "net.h"

/* #define DEBUG_EMAC */

#define EMAC_SIZE        0x4000

#define EMAC_CTL         0x00 /* Control Register */
#define EMAC_CFG         0x04
#define EMAC_SR          0x08 /* Status Register */
#define EMAC_TAR         0x0c /* Transmit Address Register */
#define EMAC_TCR         0x10 /* Transmit Control Register */
#define EMAC_TSR         0x14 /* Transmit Status Register */
#define EMAC_RBQP        0x18 /* Receive Buffer Queue Pointer */
#define EMAC_TBQP        0x1c /* Transmit Buffer Queue Pointer */
#define EMAC_RSR         0x20 /* Receive Status Register */
#define EMAC_ISR         0x24 /* Interrupt Status Register */
#define EMAC_IER         0x28 /* Interrupt Enable Register */
#define EMAC_IDR         0x2c /* Interrupt Disable Register */
#define EMAC_IMR         0x30 /* Interrupt Mask Register */
#define EMAC_MAN         0x34 /* PHY Maintenance Register */

#define PHY_CTRL         0x00 /* Control Register */
#define PHY_STATUS       0x01 /* Status Regiser */
#define PHY_ID1          0x02 /* Phy Id Reg (word 1) */
#define PHY_ID2          0x03 /* Phy Id Reg (word 2) */
#define PHY_AUTONEG_ADV  0x04 /* Autoneg Advertisement */
#define PHY_LP_ABILITY   0x05 /* Link Partner Ability (Base Page) */
#define PHY_AUTONEG_EXP  0x06 /* Autoneg Expansion Reg */
#define PHY_NEXT_PAGE_TX 0x07 /* Next Page TX */
#define PHY_LP_NEXT_PAGE 0x08 /* Link Partner Next Page */
#define PHY_1000T_CTRL   0x09 /* 1000Base-T Control Reg */
#define PHY_1000T_STATUS 0x0a /* 1000Base-T Status Reg */
#define PHY_EXT_STATUS   0x0f /* Extended Status Reg */

#define CTL_LB           0x01 /* Loopback */
#define CTL_LLB          0x02 /* Loopback local */
#define CTL_RE           0x04 /* Receive enable */
#define CTL_TE           0x08 /* Transmit enable */
#define CTL_MPE          0x10 /* Management port enable */
#define CTL_CLRSTAT      0x20 /* Clear statistics registers */
#define CTL_INCSTAT      0x40 /* Increment statistics registers */
#define CTL_WESTAT       0x80 /* Write enable for statistics registers */
#define CTL_BP           0x100 /* Back pressure */
#define CTL_TSTART       0x200 /* Transmit start */

#define CFG_SPD          0x01 /* Speed (100 or 10) */
#define CFG_FD           0x02 /* Full Duplex */
#define CFG_JFRAME       0x08 /* Jumbo Frames */
#define CFG_CAF          0x10 /* Copy All Frames */
#define CFG_NBC          0x20 /* No Broadcast */
#define CFG_MTI          0x40 /* Multicast Hash Enable */
#define CFG_UNI          0x80 /* Unicast Hash Enable */
#define CFG_BIG          0x100 /* Receive 1536 byte frames */
#define CFG_CLK          0xC00
#define CFG_CLK_HCLK_8   0 /* HCLK divided by 8 */
#define CFG_CLK_HCLK_16  0x400 /* HCLK divided by 16 */
#define CFG_CLK_HCLK_32  0x800 /* HCLK divided by 32 */
#define CFG_CLK_HCLK_64  0xc00 /* HCLK divided by 64 */
#define CFG_RTY          0x1000 /* Retry Test */
#define CFG_PAE          0x2000 /* Pause Enable */
#define CFG_RBOF         0xC000 /* Receive Buffer Offset */
#define CFG_RLCE         0x10000 /* Receive Length field Checking Enable */
#define CFG_DRFCS        0x20000 /* Discard Receive FCS */
#define CFG_EFRHD        0x40000 /* Enable Frame Receive in Half-Duplex mode */
#define CFG_IRXFCS       0x80000 /* Ignore RX FCS */

#define SR_IDLE          0x04

#define TSR_UBR          0x01 /* Used Bit Read */
#define TSR_COL          0x02 /* Collision Occured */
#define TSR_RLE          0x04 /* Retry Limit Exceeded */
#define TSR_TGO          0x08 /* Transmit Go */
#define TSR_BEX          0x10 /* Buffers exhausted mid-frame */
#define TSR_COMP         0x20 /* Transmit Complete */
#define TSR_UND          0x40 /* Transmit Underrun */

#define ISR_MFD          0x01 /* Management Frame Done */
#define ISR_RCOMP        0x02 /* Receive Completed */
#define ISR_RBNA         0x04 /* Receive Buffer Not Available */
#define ISR_TXUBR        0x08 /* Transmit Used Bit Read */
#define ISR_TUND         0x10 /* Transmit Buffer Underrun */
#define ISR_RLE          0x20 /* Retry Limit Exceeded */
#define ISR_TXERR        0x40 /* Transmit Error */
#define ISR_TCOMP        0x80 /* Transmit Complete */
#define ISR_ROVR         0x400 /* Receive Overrun */

#define TXDESC_USED      0x80000000
#define TXDESC_WRAP      0x40000000
#define TXDESC_NOCRC     0x10000
#define TXDESC_LAST      0x8000

#define CRCPOLY_LE 0xedb88320

typedef struct EMACState
{
    SysBusDevice busdev;
    qemu_irq irq;
    VLANClientState *vc;

    uint32_t ctl;
    uint32_t cfg;
    uint32_t sr;
    uint32_t tar;
    uint32_t tcr;
    uint32_t tsr;
    uint32_t rbqp;
    uint32_t rbqp_base;
    uint32_t tbqp;
    uint32_t tbqp_base;
    uint32_t rsr;
    uint32_t isr;
    uint32_t imr;
    uint32_t man;

    uint32_t hsl;
    uint16_t hsh;
    uint32_t sa1l;
    uint16_t sa1h;
    uint32_t sa2l;
    uint16_t sa2h;
    uint32_t sa3l;
    uint16_t sa3h;
    uint32_t sa4l;
    uint16_t sa4h;
    uint8_t sa_valid;
} EMACState;

static uint32_t crc32_le(uint32_t crc, unsigned char const *p, size_t len)
{
    int i;
    while (len--) {
        crc ^= *p++;
        for (i = 0; i < 8; i++)
            crc = (crc >> 1) ^ ((crc & 1) ? CRCPOLY_LE : 0);
    }
    return crc;
}

static uint32_t address_match(EMACState *s, const uint8_t *buf)
{
    uint32_t addrl = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    uint16_t addrh = (buf[4] << 8) | buf[5];

    if ((s->sa_valid & 1) && addrl == s->sa1l && addrh == s->sa1h)
        return (1 << 26);
    if ((s->sa_valid & 2) && addrl == s->sa2l && addrh == s->sa2h)
        return (1 << 25);
    if ((s->sa_valid & 4) && addrl == s->sa3l && addrh == s->sa3h)
        return (1 << 24);
    if ((s->sa_valid & 8) && addrl == s->sa4l && addrh == s->sa4h)
        return (1 << 23);
    if (!(s->cfg & CFG_NBC) && addrl == ~0 && addrh == 0xffff)
        return (1 << 31);
    /* TODO: hash addressing */

    return 0;
}

static int at91_emac_can_receive(VLANClientState *vc)
{
    EMACState *s = vc->opaque;
    return !!(s->ctl & CTL_RE);
}

static ssize_t at91_emac_receive(VLANClientState *vc, const uint8_t *buf, size_t size)
{
    EMACState *s = vc->opaque;
    uint32_t rx_desc;
    uint32_t buffer_addr;
    uint32_t status_len;
    uint32_t status_len_le;
    uint8_t segment_size;
    size_t saved_size;
    int wrap;

    if (!(s->ctl & CTL_RE))
        return -1;

    /* Reject frame with invalid size */
    if (size < 6 || size > 10240) {
        return size;
    }
    if ((size > 1536 && !(s->cfg & CFG_JFRAME)) ||
        (size > 1518 && !(s->cfg & CFG_BIG))) {
        return size;
    }

    status_len = address_match(s, buf);
    /* Either the address has to match or promiscuous mode must be enabled */
    if (status_len == 0 && !(s->cfg & CFG_CAF)) {
        return size;
    }

    /* TODO: VLAN, FCS checks (CFG_DRFCS, CFG_IRXFCS) */

    /* Start of frame */
    status_len = 1 << 14;
    saved_size = size;

    do {
        cpu_physical_memory_read(s->rbqp, (uint8_t *)&rx_desc, sizeof(uint32_t));
        rx_desc = le32_to_cpu(rx_desc);

        if (rx_desc & 1) /* Ownership bit */
        {
            s->rsr |= 1; /* Buffer Not Available */
            s->isr |= ISR_RBNA;
            if (s->imr & ISR_RBNA) {
                qemu_set_irq(s->irq, 1);
            }
            return size;
        } else {
            wrap = rx_desc & 2;
            buffer_addr = rx_desc & ~3; /* Mask out the WRAP and OWNERSHIP bits */

            segment_size = size <= 128 ? size : 128;
            cpu_physical_memory_write(buffer_addr, buf, segment_size);
            size -= segment_size;
            buf += segment_size;

            if (size == 0)
                status_len |= (1 << 15) | saved_size;
            status_len_le = cpu_to_le32(status_len);
            cpu_physical_memory_write(s->rbqp + 4, (uint8_t *)&status_len_le, sizeof(uint32_t));
            status_len &= ~(1 << 14);

            /* Set owner bit to CPU */
            rx_desc |= 1;
            rx_desc = cpu_to_le32(rx_desc);
            cpu_physical_memory_write(s->rbqp, (uint8_t *)&rx_desc, sizeof(uint32_t));

            if (wrap || s->rbqp - s->rbqp_base == 8192) {
                s->rbqp = s->rbqp_base;
            } else {
                s->rbqp = s->rbqp + 8;
            }
        }
    } while (size > 0);

    /* Frame Received */
    s->rsr |= 2;
    s->isr |= ISR_RCOMP;
    if (s->imr & ISR_RCOMP) {
        qemu_set_irq(s->irq, 1);
    }

    return size;
}

static void at91_emac_send(EMACState *s)
{
    uint32_t crc;
    uint8_t buf[2048 + sizeof(crc)];
    uint32_t size;

    size = s->tcr & 0x7ff;

    cpu_physical_memory_read(s->tar, buf, size);

    if (!(s->tcr & (1 << 15))) { /* No CRC bit */
        /* Pad the frame to minimal length */
        if (size < 60) {
            memset(buf + size, 0, 60 - size);
            size = 60;
        }

        /* Add CRC */
        crc = cpu_to_le32(~crc32_le(~0, buf, size));
        memcpy(buf + size, &crc, sizeof(crc));
        size += 4;
    }

    if (s->ctl & CTL_LLB) {
        at91_emac_receive(s->vc, buf, size);
    } else {
        qemu_send_packet(s->vc, buf, size);
    }

    /* Transfer Complete */
    s->tsr |= TSR_COMP;
    s->isr |= ISR_TCOMP;
    if (s->imr & ISR_TCOMP) {
        qemu_set_irq(s->irq, 1);
    }
}

static void at91_emac_send_queue(EMACState *s)
{
    uint32_t crc;
    uint8_t buf[2048 + sizeof(crc)];
    uint32_t buffer_addr;
    uint32_t tx_desc;
    uint32_t frame_size;
    int used;
    int wrap;

    do {
        cpu_physical_memory_read(s->tbqp, (uint8_t *)&buffer_addr, sizeof(uint32_t));
        cpu_physical_memory_read(s->tbqp + 4, (uint8_t *)&tx_desc, sizeof(uint32_t));
        buffer_addr = le32_to_cpu(buffer_addr);
        tx_desc = le32_to_cpu(tx_desc);

        used = !!(tx_desc & TXDESC_USED);
        wrap = !!(tx_desc & TXDESC_WRAP);

        if (!used) {
            frame_size = tx_desc & 0x7ff;
            cpu_physical_memory_read(le32_to_cpu(buffer_addr), buf, frame_size);

            /* TODO: Jumbo frame - !(tx_desc & TXDESC_LAST) */
            /* TODO: Pause frame */

            if (!(tx_desc & TXDESC_NOCRC)) {
                /* Pad the frame to minimal length */
                if (frame_size < 60) {
                    memset(buf + frame_size, 0, 60 - frame_size);
                    frame_size = 60;
                }

                /* Add CRC */
                crc = cpu_to_le32(~crc32_le(~0, buf, frame_size));
                memcpy(buf + frame_size, &crc, sizeof(crc));
                frame_size += 4;
            }

            if (s->ctl & CTL_LLB) {
                at91_emac_receive(s->vc, buf, frame_size);
            } else {
                qemu_send_packet(s->vc, buf, frame_size);
            }

            /* Set owner bit to CPU */
            tx_desc |= 1 << 31;
            tx_desc = cpu_to_le32(tx_desc);
            cpu_physical_memory_write(s->tbqp + 4, (uint8_t *)&tx_desc, sizeof(uint32_t));

            if (wrap || s->tbqp - s->tbqp_base == 8192) {
                s->tbqp = s->tbqp_base;
            } else {
                s->tbqp = s->tbqp + 8;
            }
        }
    } while (!used);

    /* Transfer Complete */
    s->tsr |= TSR_COMP;
    s->isr |= ISR_TCOMP;
    if (s->imr & ISR_TCOMP) {
       qemu_set_irq(s->irq, 1);
    }
}

static uint16_t at91_emac_phy_read(EMACState *s, uint8_t reg)
{
    switch (reg) {
        case PHY_ID1:
            return 0x0181; /* DM9161 */
        case PHY_ID2:
            return 0xb8a0;
        case PHY_STATUS:
            return
                (1 << 15) | /* 100BASE-T4 Capable */
                (1 << 5) | /* Auto-negotion Complete */
                (1 << 3) | /* Auto Configuration Ability */
                (1 << 2); /* Link Status */
        case PHY_AUTONEG_ADV:
            return (1 << 9) | 1; /* 100BASE-T4, IEEE 802.3 */
        default:
            return 0;
    }
}

static void at91_emac_phy_write(EMACState *s, uint8_t reg, uint16_t value)
{
}

static uint32_t at91_emac_mem_read(void *opaque, target_phys_addr_t offset)
{
    EMACState *s = opaque;
    uint32_t isr;

    offset &= EMAC_SIZE - 1;

    switch (offset) {
    case EMAC_CTL:
        return s->ctl;
    case EMAC_CFG:
        return s->cfg;
    case EMAC_SR:
        return s->sr;
    case EMAC_TAR:
        return s->tar;
    case EMAC_TCR:
        return s->tcr;
    case EMAC_TSR:
        return s->tsr;
    case EMAC_RBQP:
        return s->rbqp;
    case EMAC_TBQP:
        return s->tbqp;
    case EMAC_RSR:
        return s->rsr;
    case EMAC_ISR:
        isr = s->isr;
        s->isr = 0;
        qemu_set_irq(s->irq, 0);
        return isr;
    case EMAC_IMR:
        return s->imr;
    case EMAC_MAN:
        return s->man;
    default:
        return 0;
    }
}

static void at91_emac_mem_write(void *opaque, target_phys_addr_t offset,
                uint32_t value)
{
    EMACState *s = opaque;

    offset &= EMAC_SIZE - 1;

    switch (offset) {
    case EMAC_CTL:
        s->ctl = value & ~(CTL_CLRSTAT | CTL_INCSTAT);
        /* TODO: CTL_CLRSTAT, CTL_INCSTAT */
        if ((value & (CTL_TSTART | CTL_TE)) == (CTL_TSTART | CTL_TE)) {
            at91_emac_send_queue(s);
        } else if (!(value & CTL_TE)){
            s->tbqp = s->tbqp_base;
        }
        break;
    case EMAC_CFG:
        s->cfg = value;
        break;
    case EMAC_TAR:
        s->tar = value;
        break;
    case EMAC_TCR: /* RM9200 only */
        s->tcr = value;
        if (s->ctl & CTL_TE)
            at91_emac_send(s);
        break;
    case EMAC_TSR:
        s->tsr &= ~value;
        break;
    case EMAC_RBQP:
        s->rbqp_base = s->rbqp = value;
        break;
    case EMAC_TBQP:
        s->tbqp_base = s->tbqp = value;
        break;
    case EMAC_RSR:
        s->rsr &= ~value;
        break;
    case EMAC_ISR:
        s->isr = value;
        break;
    case EMAC_IER:
        s->imr |= value;
        break;
    case EMAC_IDR:
        s->imr &= ~value;
        break;
    case EMAC_MAN:
        /* Check for PHY Address 31 */
        if (((value >> 23) & 0x1f) == 31) {
            if ((value & 0x30000000) == 0x20000000) {
                value &= ~0xffff;
                value |= at91_emac_phy_read(s, (value >> 18) & 0x1f);
            } else if ((value & 0x30000000) == 0x10000000) {
                at91_emac_phy_write(s, (value >> 18) & 0x1f, value & 0xffff);
            }
            /* TODO: Interrupts, checks */
        }
        s->sr |= SR_IDLE;
        s->man = value;
        break;
    case 0x90:
        s->hsl = value;
        s->sa_valid &= ~16;
        break;
    case 0x94:
        s->hsh = value & 0xffff;
        s->sa_valid |= 16;
        break;
    case 0x98:
        s->sa1l = value;
        s->sa_valid &= ~1;
        break;
    case 0x9c:
        s->sa1h = value & 0xffff;
        s->sa_valid |= 1;
        break;
    case 0xa0:
        s->sa2l = value;
        s->sa_valid &= ~2;
        break;
    case 0xa4:
        s->sa2h = value & 0xffff;
        s->sa_valid |= 2;
        break;
    case 0xa8:
        s->sa3l = value;
        s->sa_valid &= ~4;
        break;
    case 0xac:
        s->sa3h = value & 0xffff;
        s->sa_valid |= 4;
        break;
    case 0xb0:
        s->sa4l = value;
        s->sa_valid &= ~8;
        break;
    case 0xb4:
        s->sa4h = value & 0xffff;
        s->sa_valid |= 8;
        break;
    default:
        return;
    }
}

#ifdef DEBUG_EMAC
static uint32_t at91_emac_mem_read_dbg(void *opaque, target_phys_addr_t offset)
{
    uint32_t value = at91_emac_mem_read(opaque, offset);
    printf("%s offset=%x val=%x\n", __func__, offset, value);
    return value;
}

static void at91_emac_mem_write_dbg(void *opaque, target_phys_addr_t offset,
                uint32_t value)
{
    printf("%s offset=%x val=%x\n", __func__, offset, value);
    at91_emac_mem_write(opaque, offset, value);
}

#define at91_emac_mem_read at91_emac_mem_read_dbg
#define at91_emac_mem_write at91_emac_mem_write_dbg
#endif

static CPUReadMemoryFunc *at91_emac_readfn[] = {
    at91_emac_mem_read,
    at91_emac_mem_read,
    at91_emac_mem_read
};

static CPUWriteMemoryFunc *at91_emac_writefn[] = {
    at91_emac_mem_write,
    at91_emac_mem_write,
    at91_emac_mem_write
};

static void at91_emac_save(QEMUFile *f, void *opaque)
{
    EMACState *s = opaque;

    qemu_put_be32(f, s->ctl);
    qemu_put_be32(f, s->cfg);
    qemu_put_be32(f, s->sr);
    qemu_put_be32(f, s->tar);
    qemu_put_be32(f, s->tcr);
    qemu_put_be32(f, s->tsr);
    qemu_put_be32(f, s->rbqp);
    qemu_put_be32(f, s->rbqp_base);
    qemu_put_be32(f, s->tbqp);
    qemu_put_be32(f, s->tbqp_base);
    qemu_put_be32(f, s->rsr);
    qemu_put_be32(f, s->isr);
    qemu_put_be32(f, s->imr);
    qemu_put_be32(f, s->man);

    qemu_put_be32(f, s->hsl);
    qemu_put_be16(f, s->hsh);
    qemu_put_be32(f, s->sa1l);
    qemu_put_be16(f, s->sa1h);
    qemu_put_be32(f, s->sa2l);
    qemu_put_be16(f, s->sa2h);
    qemu_put_be32(f, s->sa3l);
    qemu_put_be16(f, s->sa3h);
    qemu_put_be32(f, s->sa4l);
    qemu_put_be16(f, s->sa4h);
    qemu_put_byte(f, s->sa_valid);
}

static int at91_emac_load(QEMUFile *f, void *opaque, int version_id)
{
    EMACState *s = opaque;

    if (version_id != 1)
        return -EINVAL;

    s->ctl = qemu_get_be32(f);
    s->cfg = qemu_get_be32(f);
    s->sr = qemu_get_be32(f);
    s->tar = qemu_get_be32(f);
    s->tcr = qemu_get_be32(f);
    s->tsr = qemu_get_be32(f);
    s->rbqp = qemu_get_be32(f);
    s->rbqp_base = qemu_get_be32(f);
    s->tbqp = qemu_get_be32(f);
    s->tbqp_base = qemu_get_be32(f);
    s->rsr = qemu_get_be32(f);
    s->isr = qemu_get_be32(f);
    s->imr = qemu_get_be32(f);
    s->man = qemu_get_be32(f);

    s->hsl = qemu_get_be32(f);
    s->hsh = qemu_get_be16(f);
    s->sa1l = qemu_get_be32(f);
    s->sa1h = qemu_get_be16(f);
    s->sa2l = qemu_get_be32(f);
    s->sa2h = qemu_get_be16(f);
    s->sa3l = qemu_get_be32(f);
    s->sa3h = qemu_get_be16(f);
    s->sa4l = qemu_get_be32(f);
    s->sa4h = qemu_get_be16(f);
    s->sa_valid = qemu_get_byte(f);

    return 0;
}

static void at91_emac_reset(void *opaque)
{
    EMACState *s = opaque;

    s->ctl = 0;
    s->cfg = CFG_CLK_HCLK_32;
    s->sr = 0;
    s->tar = 0;
    s->tcr = 0;
    s->tsr = 0;
    s->rbqp = 0;
    s->rbqp_base = 0;
    s->tbqp = 0;
    s->tbqp_base = 0;
    s->rsr = 0;
    s->isr = 0;
    s->imr = 0x3fff;
    s->man = 0;
    s->hsl = 0;
    s->hsh = 0;
    s->sa1l = 0;
    s->sa1h = 0;
    s->sa2l = 0;
    s->sa2h = 0;
    s->sa3l = 0;
    s->sa3h = 0;
    s->sa4l = 0;
    s->sa4h = 0;
    s->sa_valid = 0;
}

static void at91_emac_init(SysBusDevice *dev)
{
    EMACState *s = FROM_SYSBUS(typeof (*s), dev);
    int mmio_index;

    sysbus_init_irq(dev, &s->irq);
    s->vc = qdev_get_vlan_client(&dev->qdev,
                                 at91_emac_can_receive,
                                 at91_emac_receive,
                                 NULL, NULL, s);
    mmio_index = cpu_register_io_memory(at91_emac_readfn,
                                        at91_emac_writefn, s);
    sysbus_init_mmio(dev, EMAC_SIZE, mmio_index);

    at91_emac_reset(s);
    qemu_register_reset(at91_emac_reset, s);

    register_savevm("at91_emac", -1, 1, at91_emac_save, at91_emac_load, s);
}

static void at91_emac_register(void)
{
    sysbus_register_dev("at91,emac", sizeof(EMACState), at91_emac_init);
}

device_init(at91_emac_register)
