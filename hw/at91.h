/*
 * AT91 Miscellaneous Definitions
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

#ifndef AT91_H
#define AT91_H 1

/* Frequency of the master clock as programmed by the Power Management
   Controller. */
extern int at91_master_clock_frequency;

typedef uint32_t (*spi_txrx_callback_fun_t)(void *opaque, uint32_t cmd, int len);

typedef struct PDCState PDCState;
typedef int (*pdc_start_transfer_t)(void *opaque,
                                     target_phys_addr_t tx,
                                     unsigned int tx_len,
                                     target_phys_addr_t rx,
                                     unsigned int rx_len,
                                     int last_transfer);

#define PDCF_ENDRX      1
#define PDCF_ENDTX      2
#define PDCF_RXFULL     4
#define PDCF_TXFULL     8
#define PDCF_NOT_ENDRX  16
#define PDCF_NOT_RXFULL 32
#define PDCF_NOT_ENDTX  64
#define PDCF_NOT_TXFULL 128

typedef void (*pdc_state_changed_t)(void *opaque, unsigned int state);

PDCState *at91_pdc_init(void *opaque, pdc_start_transfer_t start_transfer,
                            pdc_state_changed_t state_changed);
extern void at91_pdc_reset(PDCState *s);
extern void at91_pdc_write(void *opaque, target_phys_addr_t offset, uint32_t val);
extern uint32_t at91_pdc_read(void *opaque, target_phys_addr_t offset);

struct NANDFlashState;
extern void at91_nand_register(struct NANDFlashState *st);

#endif /* !AT91_H */
