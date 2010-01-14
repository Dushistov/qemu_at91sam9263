/*
 * AT91 Serial Peripheral Interface
 * Written by Evgeniy Dushistov
 * This code is licenced under the GPL.
 */
#include "sysbus.h"
#include "spi.h"
#include "at91.h"

#define SPI_SIZE 0x4000
#define SPI_CR  0x0
#define SPI_MR  0x4
#define SPI_RDR 0x8
#define SPI_TDR 0xC
#define SPI_SR  0x10
#define SPI_IER 0x14
#define SPI_IDR 0x18
#define SPI_IMR 0x1C
#define SPI_CSR0 0x30
#define SPI_CSR1 0x34
#define SPI_CSR2 0x38
#define SPI_CSR3 0x3C

#define SPI_CR_SPIEN  1
#define SPI_CR_SPIDIS 2
#define SPI_SR_ENDRX  (1 << 4)
#define SPI_SR_ENDTX  (1 << 5)
#define SPI_SR_RXBUFF (1 << 6)
#define SPI_SR_TXBUFE (1 << 7)
#define SPI_SR_SPIENS (1 << 16)

typedef struct SPIState {
    SysBusDevice busdev;
    qemu_irq irq;
    uint32_t mr;
    uint32_t rdr;
    uint32_t tdr;
    uint32_t sr;
    uint32_t imr;
    uint32_t csr[4];
    SPIControl *spi_control;
    PDCState *pdc_state;
} SPIState;

#define AT91_SPI_DEBUG
#ifdef AT91_SPI_DEBUG
#define DPRINTF(fmt, ...)                           \
    do {                                            \
        printf("AT91SPI: " fmt , ## __VA_ARGS__);    \
    } while (0)
#else
#define DPRINTF(fmt, ...) do { } while (0)
#endif

static uint32_t txrx_callback_empty_fun(void *opaque, uint32_t cmd, int len)
{
    return 0;
}

static void set_chipselect_empty(void *opaque, int on)
{
}

static const SPIControl txrx_callback_empty = {
    .txrx_callback = txrx_callback_empty_fun,
    .set_chipselect = set_chipselect_empty,
    .opaque = NULL,
};

extern CPUState *g_env;

static uint32_t at91_spi_mem_read(void *opaque, target_phys_addr_t offset)
{
    SPIState *s = opaque;

    offset &= SPI_SIZE - 1;
    DPRINTF("spi read off %X, %X\n", offset, g_env->regs[15]);

    switch (offset) {
    case SPI_MR:
        return s->mr;
    case SPI_SR:
        DPRINTF("s->sr %X\n", s->sr);
        return s->sr;
    case SPI_RDR:
        return s->rdr;
    case SPI_IMR:
        return s->imr;
    case SPI_CSR0 ... SPI_CSR3:
        return s->csr[(offset - SPI_CSR0) / sizeof(s->csr[0])];
    case 0x100 ... 0x124:
        return at91_pdc_read(s->pdc_state, offset);
    case SPI_CR:/*write only*/
    default:
        DPRINTF("unsupported offset %X\n", offset);
        return 0;
    }
}

static void at91_spi_mem_write(void *opaque, target_phys_addr_t offset,
                               uint32_t value)
{
    SPIState *s = opaque;

    offset &= SPI_SIZE - 1;
    DPRINTF("spi write off %X, val %X, %X\n", offset, value, g_env->regs[15]);
    switch (offset ){
    case SPI_CR:
        if (value & SPI_CR_SPIEN)
            s->sr |= SPI_SR_SPIENS;
        if (value & SPI_CR_SPIDIS)
            s->sr &= ~SPI_SR_SPIENS;
        break;
    case SPI_IER:
        s->imr |= value;
        break;
    case SPI_IDR:
        s->imr &= ~value;
        break;
    case SPI_MR:
        s->mr = value;
        break;
    case SPI_CSR0 ... SPI_CSR3:
        DPRINTF("bits per transfer %d\n", 8 + ((value & 0xF0) >> 4));
        s->csr[(offset - SPI_CSR0) / sizeof(s->csr[0])] = value;
        break;
    case 0x100 ... 0x124:
        at91_pdc_write(s->pdc_state, offset, value);
        break;
    default:
        DPRINTF("unsupported offset %X, val %X\n", offset, value);
        break;
    }
}

static CPUReadMemoryFunc *at91_spi_readfn[] = {
    at91_spi_mem_read,
    at91_spi_mem_read,
    at91_spi_mem_read,
};

static CPUWriteMemoryFunc *at91_spi_writefn[] = {
    at91_spi_mem_write,
    at91_spi_mem_write,
    at91_spi_mem_write,
};

static void pdc_state_changed(void *opaque, unsigned int state)
{
    SPIState *s = opaque;

    s->sr |= (state & PDCF_ENDRX) ? SPI_SR_ENDRX : 0;
    s->sr &= ~((state & PDCF_NOT_ENDRX) ? SPI_SR_ENDRX : 0);
    s->sr |= (state & PDCF_ENDTX) ? SPI_SR_ENDTX : 0;
    s->sr &= ~((state & PDCF_NOT_ENDTX) ? SPI_SR_ENDTX : 0);

    s->sr |= (state & PDCF_RXFULL) ? SPI_SR_RXBUFF : 0;
    s->sr &= ~((state & PDCF_NOT_RXFULL) ? SPI_SR_RXBUFF : 0);

    s->sr |= (state & PDCF_TXFULL) ? SPI_SR_TXBUFE : 0;
    s->sr &= ~((state & PDCF_NOT_TXFULL) ? SPI_SR_TXBUFE : 0);

    qemu_set_irq(s->irq, !!(s->sr & s->imr));
}

static int pdc_start_transfer(void *opaque,
                               target_phys_addr_t tx,
                               unsigned int tx_len,
                               target_phys_addr_t rx,
                               unsigned int rx_len,
                               int last_transfer)
{
    SPIState *s = opaque;
    unsigned int i;
    unsigned int tlen = rx_len > tx_len ? rx_len : tx_len;
    int flags = ((tx_len > 0) ? 1 : 0) | ((rx_len > 0) ? 2 : 0);

    DPRINTF("pdc: start transfer, last trans %d\n", last_transfer);
#if 1
    if (flags == 2) {
        DPRINTF("ignore only read request\n");
        return -1;
    }
#endif
    /* suppose that transfer 8 bit,
       TODO: fix this, extract right value from csr
    */
    s->spi_control->set_chipselect(s->spi_control->opaque, 1);
    for (i = 0; i < tlen; ++i) {
        DPRINTF("pdc: transfering\n");
        uint8_t tmp = 0;
        if (tx_len > 0) {
            cpu_physical_memory_read(tx, &tmp, 1);
            ++tx;
            --tx_len;
        }
        tmp = s->spi_control->txrx_callback(s->spi_control->opaque, tmp, 8);
        s->rdr = tmp;
        if (rx_len > 0) {
            cpu_physical_memory_write(rx, &tmp, 1);
            ++rx;
            --rx_len;
        }
    }
#if 0
    if (flags & 1) {//tx
        s->sr |= SPI_SR_ENDTX;
        if (last_transfer) {
            s->sr |= SPI_SR_TXBUFE;
        }
    }
    if (flags & 2) {//rx
        s->sr |= SPI_SR_ENDRX;
        if (last_transfer) {
            s->sr |= SPI_SR_RXBUFF;
        }
    }
#endif
    if (last_transfer) {        
        s->spi_control->set_chipselect(s->spi_control->opaque, 0);
    }
    return 0;
}

static void at91_spi_reset(void *opaque)
{
    SPIState *s = opaque;

    s->mr = 0;
    s->rdr = 0;
    s->sr = SPI_SR_ENDRX | SPI_SR_ENDTX | SPI_SR_RXBUFF | SPI_SR_TXBUFE;
    s->imr = 0;
    memset(s->csr, 0, sizeof(s->csr));
    at91_pdc_reset(s->pdc_state);
}

static void at91_spi_init(SysBusDevice *dev)
{
    SPIState *s = FROM_SYSBUS(typeof(*s), dev);
    int spi_regs;

    s->pdc_state = at91_pdc_init(s, pdc_start_transfer, pdc_state_changed);
    sysbus_init_irq(dev, &s->irq);
    spi_regs = cpu_register_io_memory(at91_spi_readfn, at91_spi_writefn, s);
    sysbus_init_mmio(dev, SPI_SIZE, spi_regs);
    qemu_register_reset(at91_spi_reset, s);
}

static SysBusDeviceInfo spi_info = {
    .init = at91_spi_init,
    .qdev.name = "at91,spi",
    .qdev.size = sizeof(SPIState),
    .qdev.props = (Property[]) {
        {
            .name   = "spi_control",
            .info   = &qdev_prop_ptr,
            .offset = offsetof(SPIState, spi_control),
            .defval = (void *)&txrx_callback_empty,
        },
       {/* end of list */}
    }
};


static void at91_spi_register(void)
{
    sysbus_register_withprop(&spi_info);
}

device_init(at91_spi_register)
