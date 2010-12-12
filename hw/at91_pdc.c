/*
 * AT91 Peripheral DMA Controller (PDC) User Interface
 * Written by Evgeniy Dushistov
 * This code is licenced under the GPL. 
 */
#include <stdint.h>
#include "sysbus.h"
#include "at91.h"


#define PDC_RPR  0x100          //Receive Pointer Register         Read/Write     0
#define PDC_RCR  0x104          //Receive Counter Register        PDC_RCR Read/Write     0
#define PDC_TPR  0x108          //Transmit Pointer Register       PDC_TPR Read/Write     0
#define PDC_TCR  0x10C          //Transmit Counter Register       PDC_TCR Read/Write     0
#define PDC_RNPR 0x110          //Receive Next Pointer Register  PDC_RNPR Read/Write     0
#define PDC_RNCR 0x114          //Receive Next Counter Register  PDC_RNCR Read/Write     0
#define PDC_TNPR 0x118          //Transmit Next Pointer Register PDC_TNPR Read/Write     0
#define PDC_TNCR 0x11C          //Transmit Next Counter Register PDC_TNCR Read/Write     0
#define PDC_PTCR 0x120          //Transfer Control Register      PDC_PTCR   Write        0
#define PDC_PTSR 0x124          //Transfer Status Register       PDC_PTSR   Read         0

#define PDC_PTCR_RXTEN  (1 << 0)
#define PDC_PTCR_RXTDIS (1 << 1)
#define PDC_PTCR_TXTEN  (1 << 8)
#define PDC_PTCR_TXTDIS (1 << 9)

#define AT91_PDC_DEBUG
#ifdef AT91_PDC_DEBUG
#define DPRINTF(fmt, ...)                           \
    do {                                            \
        printf("AT91PDC: " fmt , ## __VA_ARGS__);    \
    } while (0)
#else
#define DPRINTF(fmt, ...) do { } while (0)
#endif

struct PDCState {
    uint32_t ptsr;
    uint32_t rpr;
    uint32_t tpr;
    unsigned int rcr;
    unsigned int tcr;
    uint32_t rnpr;
    uint32_t tnpr;
    uint16_t rncr;
    uint16_t tncr;
    void *opaque;
    pdc_start_transfer_t start_transfer;
    pdc_state_changed_t state_changed;
};

PDCState *at91_pdc_init(void *opaque, pdc_start_transfer_t start_transfer,
                        pdc_state_changed_t state_changed)
{
    PDCState *s;

    s = qemu_mallocz(sizeof(PDCState));
    s->opaque = opaque;
    s->start_transfer = start_transfer;
    s->state_changed = state_changed;
    return s;
}

static void at91_pdc_work(PDCState *s, int last_transfer)
{
    unsigned int state = 0;

    DPRINTF("begin work\n");
    if ((s->ptsr & PDC_PTCR_RXTEN) || (s->ptsr & PDC_PTCR_TXTEN)) {
        unsigned was_rcr = s->rcr;
        unsigned was_tcr = s->tcr;
        DPRINTF("start work: tcr %u, rcr %u\n", (s->ptsr & PDC_PTCR_TXTEN) ? s->tcr : 0, 
                (s->ptsr & PDC_PTCR_RXTEN) ? s->rcr : 0);
        
        int ret = s->start_transfer(s->opaque,
                                    s->tpr, (s->ptsr & PDC_PTCR_TXTEN) ? &s->tcr : 0,
                                    s->rpr, (s->ptsr & PDC_PTCR_RXTEN) ? &s->rcr : 0,
                                    last_transfer);

        if (ret == 0) {
            if (s->ptsr & PDC_PTCR_RXTEN) {
                if (s->rcr == 0 && was_rcr) {
                    state |= PDCF_ENDRX;
                }
                s->rpr += was_rcr - s->rcr;
                DPRINTF("rpr(1) %X\n", s->rpr);
            }
            if (s->ptsr & PDC_PTCR_TXTEN) {
                if (s->tcr == 0 && was_tcr) {
                    state |= PDCF_ENDTX;
                }
                s->tpr += was_tcr - s->tcr;
            }

        } else
            return;


        if (last_transfer == 0) {
            if ((s->ptsr & PDC_PTCR_RXTEN) && s->rcr == 0 && s->rncr != 0) {
                s->rpr = s->rnpr;
                s->rcr = s->rncr;
                s->rncr = 0;
            }
            if ((s->ptsr & PDC_PTCR_TXTEN) && s->tcr == 0 && s->tncr) {
                s->tpr = s->tnpr;
                s->tcr = s->tncr;
                s->tncr = 0;
            }
            was_tcr = s->tcr;
            was_rcr = s->rcr;
            s->start_transfer(s->opaque, s->tpr,
                              (s->ptsr & PDC_PTCR_TXTEN) ? &s->tcr : 0,
                              s->rpr, (s->ptsr & PDC_PTCR_RXTEN) ? &s->rcr : 0, 1);

            if (s->ptsr & PDC_PTCR_RXTEN) {
                s->rpr += was_rcr - s->rcr;
                DPRINTF("rpr(2) %X\n", s->rpr);
            }
            if (s->ptsr & PDC_PTCR_TXTEN) {
                s->tpr += was_tcr - s->tcr;
            }
        }
    }
    if (s->rcr == 0 && s->rncr == 0) {
        state |= PDCF_RXFULL;
    }
    if (s->tcr == 0 && s->tncr == 0) {
        state |= PDCF_TXFULL;
    }
    if (state != 0) {
        s->state_changed(s->opaque, state);
    }
}

int at91_pdc_byte_in(PDCState *s, uint8_t data)
{
    DPRINTF("bytein begin rx en %d\n", s->ptsr & PDC_PTCR_RXTEN);
    DPRINTF("bytein data `%c', rcr %u\n", data, s->rcr);
    if (s->ptsr & PDC_PTCR_RXTEN) {
        if (s->rcr) {
            DPRINTF("we save `%c' to %X\n", data, s->rpr);
            cpu_physical_memory_write(s->rpr, &data, 1);
            --s->rcr;
            ++s->rpr;
            DPRINTF("s->rpr(3) %X\n", s->rpr);
            if (s->rcr == 0) {
                s->state_changed(s->opaque, PDCF_ENDRX);
                return 1;
            }
        }
    }
    return 0;
}

void at91_pdc_write(void *opaque, target_phys_addr_t offset, uint32_t val)
{
    PDCState *s = opaque;
    int last_transfer = 1;

    switch (offset) {
    case PDC_PTCR:
        DPRINTF("%s, %s was (%s, %s)\n",
                val & PDC_PTCR_RXTEN ? "enable recieve" : "<>", 
                val & PDC_PTCR_TXTEN ? "enable transfer" : "<>",
                s->ptsr & PDC_PTCR_RXTEN ? "enable recieve" : "<>", 
                s->ptsr & PDC_PTCR_TXTEN ? "enable transfer" : "<>");

        s->ptsr |= (val & PDC_PTCR_RXTEN) ? PDC_PTCR_RXTEN : 0;
        s->ptsr &= ~((val & PDC_PTCR_RXTDIS) ? PDC_PTCR_RXTEN : 0);

        if (s->ptsr & PDC_PTCR_RXTEN) {
          last_transfer = s->rncr == 0;
        }

        s->ptsr |= (val & PDC_PTCR_TXTEN) ? PDC_PTCR_TXTEN : 0;
        s->ptsr &= ~((val & PDC_PTCR_TXTDIS) ? PDC_PTCR_TXTEN : 0);

        if (s->ptsr & PDC_PTCR_TXTEN) {
            last_transfer |= s->tncr == 0;
        }
               
        at91_pdc_work(s, last_transfer);

        break;
    case PDC_RPR:
        s->rpr = val;
        break;
    case PDC_TPR:
        s->tpr = val;
        break;
    case PDC_RCR:
        s->rcr = val & 0xFFFF;
        if (s->rcr != 0) {
            s->state_changed(s->opaque, PDCF_NOT_ENDRX | PDCF_NOT_RXFULL);
#if 1
            if ((s->ptsr & PDC_PTCR_RXTEN)) {
                DPRINTF("rcr start workn\n");
                at91_pdc_work(s, 0);
            }
#endif
        }
        break;
    case PDC_TCR:
        s->tcr = val & 0xFFFF;
        if (s->tcr != 0) {
            s->state_changed(s->opaque, PDCF_NOT_ENDTX | PDCF_NOT_TXFULL);
            if ((s->ptsr & PDC_PTCR_TXTEN)) {
                DPRINTF("tcr start workn\n");
                at91_pdc_work(s, 0);
            }
        }
        break;
    case PDC_RNPR:
        s->rnpr = val;
        break;
    case PDC_RNCR:
        s->rncr = val;
        if (s->rncr != 0) {
            s->state_changed(s->opaque, PDCF_NOT_RXFULL);
        }
        break;
    case PDC_TNPR:
        s->tnpr = val;
        break;
    case PDC_TNCR:
        s->tncr = val;
        if (s->tncr != 0) {
            s->state_changed(s->opaque, PDCF_NOT_TXFULL);
        }
        break;
    default:
        DPRINTF("ignore write of %X to %X\n", val, offset);
        /*ignore*/break;
    }
}

uint32_t at91_pdc_read(void *opaque, target_phys_addr_t offset)
{
    PDCState *s = opaque;
    DPRINTF("read from %X\n", offset);
    switch (offset) {
    case PDC_RPR:
        return s->rpr;
    case PDC_TPR:
        return s->tpr;
    case PDC_RCR:
        return s->rcr & 0xFFFF;
    case PDC_TCR:
        return s->tcr & 0xFFFF;
    case PDC_RNPR:
        return s->rnpr;
    case PDC_RNCR:
        return s->rncr;
    case PDC_TNPR:
        return s->tnpr;
    case PDC_TNCR:
        return s->tncr;
    default:
        DPRINTF("ignore read from %X\n", offset);
    }
    return 0;
}

void at91_pdc_reset(PDCState *s)
{
    s->ptsr = 0;
    s->rpr = 0;
    s->tpr = 0;
    s->rcr = 0;
    s->tcr = 0;

    s->tnpr = 0;
    s->rnpr = 0;
    s->tncr = 0;
    s->rncr = 0;
}
