/*
 * Atmel at91sam9263 cpu + NOR flash + SDRAM emulation
 * Written by Evgeniy Dushistov
 * This code is licenced under the GPL.
 */

#include <stdio.h>
#include <stdlib.h>

#include "hw.h"
#include "arm-misc.h"
#include "primecell.h"
#include "devices.h"
#include "net.h"
#include "sysemu.h"
#include "flash.h"
#include "boards.h"
#include "qemu-char.h"
#include "qemu-timer.h"

#include "at91sam9263_defs.h"

//#define AT91_TRACE_ON
//#define AT91_DEBUG_ON

#define ARM_AIC_CPU_IRQ 0
#define ARM_AIC_CPU_FIQ 1

#define NOR_FLASH_SIZE (1024 * 1024 * 8)
#define AT91SAM9263EK_SDRAM_SIZE (1024 * 1024 * 64)
#define AT91SAM9263EK_SDRAM_OFF 0x20000000
#define AT91SAM9263EK_NORFLASH_OFF 0x10000000

#ifdef AT91_DEBUG_ON
#       define DEBUG(f, a...)    {              \
        printf ("at91 (%s, %d): %s:",           \
                __FILE__, __LINE__, __func__);  \
        printf (f, ## a);                       \
    }
#else
#       define DEBUG(f, a...) do { } while (0)
#endif

#ifdef AT91_TRACE_ON
static FILE *trace_file;
#       define TRACE(f, a...)    do {           \
        fprintf (trace_file, f, ## a);          \
    } while (0)
#else
#       define TRACE(f, a...) do { } while (0)
#endif


struct dbgu_state {
    CharDriverState *chr;
};

struct at91sam9_state {
    uint32_t pmc_regs[28];
    uint32_t dbgu_regs[0x124 / 4];
    uint32_t bus_matrix_regs[0x100 / 4];
    uint32_t ccfg_regs[(0x01FC - 0x0110 + 1) / sizeof(uint32_t)];
    uint32_t pio_regs[(AT91_PMC_BASE - AT91_PIO_BASE) / sizeof(uint32_t)];
    uint32_t sdramc0_regs[(AT91_SMC0_BASE - AT91_SDRAMC0_BASE) / sizeof(uint32_t)];
    uint32_t smc0_regs[(AT91_ECC1_BASE - AT91_SMC0_BASE) / sizeof(uint32_t)];
    uint32_t pitc_regs[80];
    uint32_t aic_regs[(AT91_PIO_BASE - AT91_AIC_BASE) / sizeof(uint32_t)];
    uint32_t usart0_regs[0x1000 / sizeof(uint32_t)];
    struct dbgu_state dbgu;
    pflash_t *norflash;
    ram_addr_t internal_sram;
    QEMUTimer *dbgu_tr_timer;
    ptimer_state *pitc_timer;
    int timer_active;
    CPUState *env;
    qemu_irq *qirq;
    uint64_t char_transmit_time;
};

static uint32_t at91_pmc_read(void *opaque, target_phys_addr_t offset)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;
    TRACE("pmc read offset %X\n", offset);
    return sam9->pmc_regs[offset / 4];
}

static void at91_pmc_write(void *opaque, target_phys_addr_t offset,
                           uint32_t value)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;

    TRACE("pmc write offset %X, value %X\n", offset, value);
    switch (offset / 4) {
    case AT91_PMC_MCKR:
        sam9->pmc_regs[AT91_PMC_MCKR] = value;
        switch (value & AT91_PMC_CSS) {
        case 1:
            //Main clock is selected
            sam9->pmc_regs[AT91_PMC_SR] |= AT91_PMC_MCKRDY | AT91_PMC_MOSCS;
            break;
        default:
            DEBUG("unimplemented\n");
            break;
        }
        break;
    case AT91_PMC_MOR:
        sam9->pmc_regs[AT91_PMC_MOR] = value;
        if (value & 1) {
            sam9->pmc_regs[AT91_PMC_SR] |= AT91_PMC_MOSCS;
        }
        break;
    case AT91_PMC_PLLAR:
        sam9->pmc_regs[AT91_PMC_PLLAR] = value;
        sam9->pmc_regs[AT91_PMC_SR] |= AT91_PMC_LOCKA;
        break;
    case AT91_PMC_PLLBR:
        sam9->pmc_regs[AT91_PMC_PLLBR] = value;
        sam9->pmc_regs[AT91_PMC_SR] |= AT91_PMC_LOCKB;
        break;
    case AT91_PMC_PCER:
        sam9->pmc_regs[AT91_PMC_PCER] |= value;
        break;
    default:
        //DEBUG("unimplemented, offset %X\n", offset);
        break;
    }
}

static uint32_t at91_dbgu_read(void *opaque, target_phys_addr_t offset)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;

    offset /= 4;
    if (offset == AT91_DBGU_RHR) {
        sam9->dbgu_regs[AT91_DBGU_SR] &= (uint32_t)~1;
    }/* else if (offset == AT91_DBGU_SR)*/

    return sam9->dbgu_regs[offset];
}

static void at91_dbgu_state_changed(struct at91sam9_state *sam9)
{
    if ((sam9->aic_regs[AT91_AIC_IECR] & (1 << AT91_PERIPH_SYS_ID)) &&
        (sam9->dbgu_regs[AT91_DBGU_IMR] & sam9->dbgu_regs[AT91_DBGU_SR])) {
        sam9->aic_regs[AT91_AIC_ISR] = AT91_PERIPH_SYS_ID;
        sam9->aic_regs[AT91_AIC_IVR] = sam9->aic_regs[AT91_AIC_SVR0 + AT91_PERIPH_SYS_ID];
        qemu_irq_raise(sam9->qirq[0]);
    }
}

static void dbgu_serial_end_xmit(void *opaque)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;
    sam9->dbgu_regs[AT91_DBGU_SR] |= (AT91_US_TXEMPTY | AT91_US_TXRDY);
    at91_dbgu_state_changed(sam9);
}

static void at91_dbgu_write(void *opaque, target_phys_addr_t offset,
                            uint32_t value)
{
    unsigned char ch;
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;

    switch (offset / 4) {
    case AT91_DBGU_CR:
        sam9->dbgu_regs[AT91_DBGU_CR] = value;
        if (value & AT91_US_TXEN)
            sam9->dbgu_regs[AT91_DBGU_SR] |= AT91_US_TXRDY | AT91_US_TXEMPTY;
        if (value & AT91_US_TXDIS)
            sam9->dbgu_regs[AT91_DBGU_SR] &= ~(AT91_US_TXRDY | AT91_US_TXEMPTY);
        break;
    case AT91_DBGU_IER:
        sam9->dbgu_regs[AT91_DBGU_IMR] |= value;
        at91_dbgu_state_changed(sam9);
        break;
    case AT91_DBGU_IDR:
        sam9->dbgu_regs[AT91_DBGU_IMR] &= ~value;
        break;
    case AT91_DBGU_THR:
        sam9->dbgu_regs[AT91_DBGU_THR] = value;
        sam9->dbgu_regs[AT91_DBGU_SR] &= ~(AT91_US_TXEMPTY | AT91_US_TXRDY);
        ch = value;
        if (sam9->dbgu.chr)
            qemu_chr_write(sam9->dbgu.chr, &ch, 1);
//        qemu_mod_timer(sam9->dbgu_tr_timer , qemu_get_clock(vm_clock) + sam9->char_transmit_time);
	dbgu_serial_end_xmit(sam9);
        break;
    default:
        //DEBUG("unimplemented\n");
        break;
    }
}

static int at91_dbgu_can_receive(void *opaque)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;

    return (sam9->dbgu_regs[AT91_DBGU_SR] & AT91_US_RXRDY) == 0;
}

static void at91_dbgu_receive(void *opaque, const uint8_t *buf, int size)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;
    int i;
    /*! \todo if not one character we need wait before irq handled,
      but from other hand at91_dbgu_can_receive should prevent this
     */
    for (i = 0; i < size; ++i) {
        sam9->dbgu_regs[AT91_DBGU_RHR] = buf[i];
        sam9->dbgu_regs[AT91_DBGU_SR] |= AT91_US_RXRDY;
        at91_dbgu_state_changed(sam9);
    }
}

static void at91_dbgu_event(void *opaque, int event)
{
    DEBUG("begin\n");
}

static uint32_t at91_bus_matrix_read(void *opaque, target_phys_addr_t offset)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;

    TRACE("bus_matrix read offset %X\n", offset);
    return sam9->bus_matrix_regs[offset / 4];
}

static void at91_bus_matrix_write(void *opaque, target_phys_addr_t offset,
                                  uint32_t value)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;

    TRACE("bus_matrix write offset %X, value %X\n", offset, value);
    switch (offset) {
    case MATRIX_MRCR:
        DEBUG("write to MATRIX mrcr reg\n");
        if (value & (AT91C_MATRIX_RCA926I | AT91C_MATRIX_RCA926D)) {
            cpu_register_physical_memory(0x0, 80 * 1024, sam9->internal_sram | IO_MEM_RAM);
        }
        break;
    default:
        DEBUG("unimplemented\n");
        break;
    }
}

static void at91_init_pmc(struct at91sam9_state *sam9)
{
    DEBUG("begin\n");
    sam9->pmc_regs[AT91_PMC_MCKR] = 0;
    sam9->pmc_regs[AT91_PMC_MOR] = 0;
    sam9->pmc_regs[AT91_PMC_SR] = 0x08;
    sam9->pmc_regs[AT91_PMC_PLLAR] = 0x3F00;
    sam9->pmc_regs[AT91_PMC_PLLBR] = 0x3F00;
}

static void at91_init_bus_matrix(struct at91sam9_state *sam9)
{
    DEBUG("begin\n");
    memset(&sam9->bus_matrix_regs, 0, sizeof(sam9->bus_matrix_regs));
}

static void at91_init_dbgu(struct at91sam9_state *sam9, CharDriverState *chr)
{
    DEBUG("begin\n");

    memset(&sam9->dbgu_regs, 0, sizeof(sam9->dbgu_regs));
    sam9->dbgu_regs[AT91_DBGU_SR] = AT91_US_TXEMPTY | AT91_US_TXRDY;

    sam9->dbgu.chr = chr;
    sam9->dbgu_tr_timer = qemu_new_timer(vm_clock, (QEMUTimerCB *)dbgu_serial_end_xmit, sam9);
//    sam9->char_transmit_time = (get_ticks_per_sec() / 115200) * 10;
    if (chr)
        qemu_chr_add_handlers(chr, at91_dbgu_can_receive,
                              at91_dbgu_receive,
                              at91_dbgu_event, sam9);
}

static uint32_t at91_ccfg_read(void *opaque, target_phys_addr_t offset)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;

    TRACE("ccfg read offset %X\n", offset);
    return sam9->ccfg_regs[offset / sizeof(sam9->ccfg_regs[0])];
}

static void at91_ccfg_write(void *opaque, target_phys_addr_t offset,
                            uint32_t value)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;

    TRACE("ccfg write offset %X, value %X\n", offset, value);
    sam9->ccfg_regs[offset / sizeof(sam9->ccfg_regs[0])] = value;
}

static uint32_t at91_pio_read(void *opaque, target_phys_addr_t offset)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;

    TRACE("pio read offset %X\n", offset);
    return sam9->pio_regs[offset / sizeof(sam9->pio_regs[0])];
}

static void at91_pio_write(void *opaque, target_phys_addr_t offset,
                           uint32_t value)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;

    TRACE("pio write offset %X, value %X\n", offset, value);
    sam9->pio_regs[offset / sizeof(sam9->pio_regs[0])] = value;
}

static uint32_t at91_sdramc0_read(void *opaque, target_phys_addr_t offset)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;
 
    TRACE("sdramc0 read offset %X\n", offset);
    return sam9->sdramc0_regs[offset / sizeof(sam9->sdramc0_regs[0])];
}

static void at91_sdramc0_write(void *opaque, target_phys_addr_t offset,
                               uint32_t value)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;
 
    TRACE("sdramc0 write offset %X, value %X\n", offset, value);
    sam9->sdramc0_regs[offset / sizeof(sam9->sdramc0_regs[0])] = value;
}

static uint32_t at91_smc0_read(void *opaque, target_phys_addr_t offset)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;

    TRACE("smc0 read offset %X\n", offset);
    return sam9->smc0_regs[offset / sizeof(sam9->smc0_regs[0])];
}

static void at91_smc0_write(void *opaque, target_phys_addr_t offset,
                            uint32_t value)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;

    TRACE("smc0 write offset %X, value %X\n", offset, value);
    sam9->smc0_regs[offset / sizeof(sam9->smc0_regs[0])] = value;
}

static void at91_pitc_timer_tick(void * opaque)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;
    uint64_t val = ptimer_get_count(sam9->pitc_timer);

    unsigned int tick = (sam9->pitc_regs[AT91_PITC_PIIR] >> 20) + 1;
    sam9->pitc_regs[AT91_PITC_PIIR] = (val & ((1 << 21) - 1)) | (tick << 20);
    sam9->pitc_regs[AT91_PITC_PIVR] = sam9->pitc_regs[AT91_PITC_PIIR];

    if ((sam9->pitc_regs[AT91_PITC_MR] & AT91_PTIC_MR_PITIEN) &&
        (sam9->aic_regs[AT91_AIC_IECR] & (1 << AT91_PERIPH_SYS_ID)) &&
        !sam9->pitc_regs[AT91_PITC_SR]) {
        sam9->aic_regs[AT91_AIC_ISR] = AT91_PERIPH_SYS_ID;
        sam9->aic_regs[AT91_AIC_IVR] = sam9->aic_regs[AT91_AIC_SVR0 + AT91_PERIPH_SYS_ID];

        sam9->pitc_regs[AT91_PITC_SR] = 1;
        qemu_irq_raise(sam9->qirq[0]);
    }
}

static uint32_t at91_pitc_read(void *opaque, target_phys_addr_t offset)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;
    int pitc_enable = !!(sam9->pitc_regs[AT91_PITC_MR] & AT91_PTIC_MR_PITEN);
    int idx;

    idx = offset / sizeof(sam9->pitc_regs[0]);
    switch (idx) {
    case AT91_PITC_SR:
//        DEBUG("read sr: %X\n", sam9->pitc_regs[idx]);
        break;
    case AT91_PITC_PIVR:
        if (pitc_enable) {
//            DEBUG("clear sr\n");
            sam9->pitc_regs[AT91_PITC_SR] = 0;
        } else {
//            DEBUG("pitc disabled\n");
            break;
        }
    case AT91_PITC_PIIR:
        if (pitc_enable) {
            uint64_t val = ptimer_get_count(sam9->pitc_timer);
            unsigned int tick = (sam9->pitc_regs[AT91_PITC_PIIR] >> 20);
            uint32_t res = (tick << 20) | (val & ((1 << 21) - 1));

            if (idx == AT91_PITC_PIVR)
                tick = 0;
            sam9->pitc_regs[AT91_PITC_PIIR] = (tick << 20) | (val & ((1 << 21) - 1));
            sam9->pitc_regs[AT91_PITC_PIVR] = sam9->pitc_regs[AT91_PITC_PIIR];
            TRACE("pitc read offset %X, value %X\n", offset, res);
            return res;
        } else {
//            DEBUG("pitc disabled\n");
            break;
        }

    default:
        /*nothing*/break;
    }
    TRACE("pitc read offset %X, value %X\n", offset, sam9->pitc_regs[idx]);

    return sam9->pitc_regs[idx];
}

static void at91_pitc_write(void *opaque, target_phys_addr_t offset,
                            uint32_t value)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;

    TRACE("pitc write offset %X, value %X\n", offset, value);
    int idx = offset / sizeof(sam9->pitc_regs[0]);
    switch (idx) {
    case AT91_PITC_MR: {
        int pitc_enable = !!(value & AT91_PTIC_MR_PITEN);
        unsigned int period = value & 0xFFFFF;

        //DEBUG("set period %u, int enabled? %d\n", period, !!(value & AT91_PTIC_MR_PITIEN));

        if (pitc_enable && period != 0) {
            sam9->timer_active = 1;
            //! \todo get real value from PLL
            ptimer_set_freq(sam9->pitc_timer, (100 * 1000 * 1000) / 16);
            ptimer_set_limit(sam9->pitc_timer, period, 1);
            ptimer_run(sam9->pitc_timer, 0);
        } else if (sam9->timer_active) {
            TRACE("disable timer\n");
            sam9->pitc_regs[AT91_PITC_PIVR] = 0;
            ptimer_stop(sam9->pitc_timer);
        }
    }
        break;
    default:
        TRACE("unhandled register %d\n", idx);
        break;
    }
    sam9->pitc_regs[idx] = value;
}

static void at91_init_pitc(struct at91sam9_state *sam9)
{
    QEMUBH *bh;

    DEBUG("begin\n");
    memset(&sam9->pitc_regs, 0, sizeof(sam9->pitc_regs));
    sam9->pitc_regs[AT91_PITC_MR] = 0xFFFFF;
    bh = qemu_bh_new(at91_pitc_timer_tick, sam9);
    sam9->pitc_timer = ptimer_init(bh);
}

static uint32_t at91_aic_read(void *opaque, target_phys_addr_t offset)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;
    unsigned int idx = offset / sizeof(sam9->aic_regs[0]);
    if (idx == AT91_AIC_IVR) {
        qemu_irq_lower(sam9->qirq[0]);
    } else if (idx == AT91_AIC_ISR) {
        uint32_t val = sam9->aic_regs[idx];
        sam9->aic_regs[idx] = 0;
        return val;
    }

    return sam9->aic_regs[idx];
}

static void at91_aic_write(void *opaque, target_phys_addr_t offset,
                           uint32_t value)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;
    unsigned int idx = offset / sizeof(sam9->aic_regs[0]);

    switch (idx) {
    case AT91_AIC_IECR:
        sam9->aic_regs[idx] |= value;
        break;
    case AT91_AIC_IDCR:
        sam9->aic_regs[idx] |= value;
        sam9->aic_regs[AT91_AIC_IECR] &= ~value;
        break;
    case AT91_AIC_IVR://ignore write
        break;
    case AT91_AIC_EOICR:
//        DEBUG("we write to end of interrupt reg\n");
        break;
    default:
        sam9->aic_regs[idx] = value;
        break;
    }
}

static uint32_t at91_usart_read(void *opaque, target_phys_addr_t offset)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;
    DEBUG("we read from %X\n", offset);
    return sam9->usart0_regs[offset / sizeof(uint32_t)];
}

static void at91_usart_write(void *opaque, target_phys_addr_t offset,
                             uint32_t value)
{
    struct at91sam9_state *sam9 = (struct at91sam9_state *)opaque;
    DEBUG("we write to %X: %X\n", offset, value);
    sam9->usart0_regs[offset / sizeof(uint32_t)] = value;
}


struct ip_block {
    target_phys_addr_t offset;
    target_phys_addr_t end_offset;
    uint32_t (*read_func)(void *, target_phys_addr_t);
    void (*write_func)(void *, target_phys_addr_t, uint32_t);
};

static struct ip_block ip_blocks[] = {
    {AT91_USART0_BASE - AT91_PERIPH_BASE, AT91_USART0_BASE - AT91_PERIPH_BASE + 0x1000, at91_usart_read, at91_usart_write},
    {AT91_SDRAMC0_BASE - AT91_PERIPH_BASE, AT91_SMC0_BASE - AT91_PERIPH_BASE, at91_sdramc0_read, at91_sdramc0_write},
    {AT91_SMC0_BASE - AT91_PERIPH_BASE, AT91_ECC1_BASE - AT91_PERIPH_BASE, at91_smc0_read, at91_smc0_write},
    {AT91_BUS_MATRIX_BASE - AT91_PERIPH_BASE, AT91_CCFG_BASE - AT91_PERIPH_BASE, at91_bus_matrix_read, at91_bus_matrix_write},
    {AT91_CCFG_BASE - AT91_PERIPH_BASE, AT91_DBGU_BASE - AT91_PERIPH_BASE, at91_ccfg_read, at91_ccfg_write},
    {AT91_DBGU_BASE - AT91_PERIPH_BASE, AT91_AIC_BASE - AT91_PERIPH_BASE, at91_dbgu_read, at91_dbgu_write},
    {AT91_AIC_BASE - AT91_PERIPH_BASE, AT91_PIO_BASE - AT91_PERIPH_BASE, at91_aic_read, at91_aic_write},
    {AT91_PIO_BASE - AT91_PERIPH_BASE, AT91_PMC_BASE - AT91_PERIPH_BASE, at91_pio_read, at91_pio_write},
    {AT91_PMC_BASE - AT91_PERIPH_BASE, AT91_RSTC_BASE - AT91_PERIPH_BASE, at91_pmc_read, at91_pmc_write},
    {AT91_PITC_BASE - AT91_PERIPH_BASE, AT91_WDT_BASE - AT91_PERIPH_BASE, at91_pitc_read, at91_pitc_write},
};

static uint32_t at91_periph_read(void *opaque, target_phys_addr_t offset)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(ip_blocks); ++i)
        if (offset >= ip_blocks[i].offset && offset < ip_blocks[i].end_offset)
            return ip_blocks[i].read_func(opaque, offset - ip_blocks[i].offset);
    DEBUG("read from unsupported periph addr %X\n", offset);
    return 0xFFFFFFFFUL;
}

static void at91_periph_write(void *opaque, target_phys_addr_t offset,
                              uint32_t value)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(ip_blocks); ++i)
        if (offset >= ip_blocks[i].offset && offset < ip_blocks[i].end_offset) {
            ip_blocks[i].write_func(opaque, offset - ip_blocks[i].offset, value);
            return;
        }
    DEBUG("write to unsupported periph: addr %X, val %X\n", offset, value);
}

/* Input 0 is IRQ and input 1 is FIQ.  */
static void arm_aic_cpu_handler(void *opaque, int irq, int level)
{
    CPUState *env = (CPUState *)opaque;
    switch (irq) {
    case ARM_AIC_CPU_IRQ:
        if (level)
            cpu_interrupt(env, CPU_INTERRUPT_HARD);
        else
            cpu_reset_interrupt(env, CPU_INTERRUPT_HARD);
        break;
    case ARM_AIC_CPU_FIQ:
        if (level)
            cpu_interrupt(env, CPU_INTERRUPT_FIQ);
        else
            cpu_reset_interrupt(env, CPU_INTERRUPT_FIQ);
        break;
    default:
        hw_error("arm_pic_cpu_handler: Bad interrput line %d\n", irq);
    }
}

static void at91_aic_init(struct at91sam9_state *sam9)
{
    memset(&sam9->aic_regs[0], 0, sizeof(sam9->aic_regs));
    sam9->qirq = qemu_allocate_irqs(arm_aic_cpu_handler, sam9->env, 2);
}

static CPUReadMemoryFunc *at91_periph_readfn[] = {
    at91_periph_read,
    at91_periph_read,
    at91_periph_read
};

static CPUWriteMemoryFunc *at91_periph_writefn[] = {
    at91_periph_write,
    at91_periph_write,
    at91_periph_write
};


static void at91sam9_init(ram_addr_t ram_size,
                          const char *boot_device,
                          const char *kernel_filename,
                          const char *kernel_cmdline,
                          const char *initrd_filename,
                          const char *cpu_model)
{
    CPUState *env;
    DriveInfo *dinfo;
    struct at91sam9_state *sam9;
    int iomemtype;

    DEBUG("begin, ram_size %llu\n", (unsigned long long)ram_size);
#ifdef TRACE_ON
    trace_file = fopen("/tmp/trace.log", "w");
#endif
    if (!cpu_model)
        cpu_model = "arm926";
    env = cpu_init(cpu_model);
    if (!env) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(EXIT_FAILURE);
    }
    /* SDRAM at chipselect 1.  */
    cpu_register_physical_memory(AT91SAM9263EK_SDRAM_OFF, AT91SAM9263EK_SDRAM_SIZE,
                                 qemu_ram_alloc(AT91SAM9263EK_SDRAM_SIZE) | IO_MEM_RAM);

    sam9 = (struct at91sam9_state *)qemu_mallocz(sizeof(*sam9));
    if (!sam9) {
        fprintf(stderr, "allocation failed\n");
        exit(EXIT_FAILURE);
    }
    sam9->env = env;
    /* Internal SRAM */
    sam9->internal_sram = qemu_ram_alloc(80 * 1024);
    cpu_register_physical_memory(0x00300000, 80 * 1024, sam9->internal_sram | IO_MEM_RAM);

    /*Internal Peripherals */
    iomemtype = cpu_register_io_memory(at91_periph_readfn, at91_periph_writefn, sam9);
    cpu_register_physical_memory(0xF0000000, 0xFFFFFFFF - 0xF0000000, iomemtype);

    at91_init_pmc(sam9);
    at91_init_bus_matrix(sam9);
    memset(&sam9->ccfg_regs, 0, sizeof(sam9->ccfg_regs));
    memset(&sam9->pio_regs, 0, sizeof(sam9->pio_regs));
    memset(&sam9->sdramc0_regs, 0, sizeof(sam9->sdramc0_regs));
    memset(&sam9->smc0_regs, 0, sizeof(sam9->smc0_regs));
    at91_init_dbgu(sam9, serial_hds[0]);
    at91_init_pitc(sam9);
    at91_aic_init(sam9);
    memset(sam9->usart0_regs, 0, sizeof(sam9->usart0_regs));

    /*
      we use variant of booting from external memory (NOR FLASH),
      it mapped to 0x0 at start, and also it is accessable from 0x10000000 address
    */
    dinfo = drive_get(IF_PFLASH, 0, 0);
    if (dinfo) {
        ram_addr_t nor_flash_mem = qemu_ram_alloc(NOR_FLASH_SIZE);
        if (!nor_flash_mem) {
            fprintf(stderr, "allocation failed\n");
            exit(EXIT_FAILURE);
        }

        sam9->norflash = pflash_cfi_atmel_register(AT91SAM9263EK_NORFLASH_OFF,
                                                   nor_flash_mem,
                                                   dinfo->bdrv,
                                                   4 * 1024 * 2, 8,
                                                   32 * 1024 * 2,
                                                   (135 - 8),
                                                   2, 0x001F, 0x01D6, 0, 0);

        if (!sam9->norflash) {
            fprintf(stderr, "qemu: error registering flash memory.\n");
            exit(EXIT_FAILURE);
        }

        DEBUG("register flash at 0x0\n");
        //register only part of flash, to prevent conflict with internal sram
        cpu_register_physical_memory(0x0, 100 * 1024 /*NOR_FLASH_SIZE*/,
                                     nor_flash_mem | IO_MEM_ROMD);
    } else {
        fprintf(stderr, "qemu: can not start without flash.\n");
        exit(EXIT_FAILURE);
    }
    env->regs[15] = 0x0;
}

static QEMUMachine at91sam9263ek_machine = {
    .name = "at91sam9263ek",
    .desc = "Atmel at91sam9263ek board (ARM926EJ-S)",
    .init = at91sam9_init,
};

static void at91sam9_machine_init(void)
{
    qemu_register_machine(&at91sam9263ek_machine);
}

machine_init(at91sam9_machine_init)
