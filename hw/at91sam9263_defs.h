#ifndef _HW_AT91SAM9263_DEFS_H_
#define _HW_AT91SAM9263_DEFS_H_

/* base periph addresses */
#define AT91_LCDC_BASE        0x00700000
#define AT91_PERIPH_BASE     0xF0000000
#define AT91_TC012_BASE      0xFFF7C000
#define AT91_USART0_BASE     0xFFF8C000
#define AT91_EMAC_BASE       0xFFFBC000
#define AT91_SPI0_BASE       0xFFFA4000
#define AT91_SDRAMC0_BASE    0xFFFFE200
#define AT91_SMC0_BASE       0xFFFFE400
#define AT91_ECC1_BASE       0xFFFFE600
#define AT91_BUS_MATRIX_BASE 0xFFFFEC00
#define AT91_CCFG_BASE       0xFFFFED10
#define AT91_DBGU_BASE       0xFFFFEE00
#define AT91_AIC_BASE        0xFFFFF000
#define AT91_PIOA_BASE       0xFFFFF200
#define AT91_PIOB_BASE       0xFFFFF400
#define AT91_PIOC_BASE       0xFFFFF600
#define AT91_PIOD_BASE       0xFFFFF800
#define AT91_PIOE_BASE       0xFFFFFA00
#define AT91_PMC_BASE        0xFFFFFC00
#define AT91_RSTC_BASE       0xFFFFFD00
#define AT91_PITC_BASE       0xFFFFFD30
#define AT91_WDT_BASE        0xFFFFFD40

/* PMC registers */
#define AT91_PMC_SR (0x68 / sizeof(uint32_t))
#define AT91_PMC_MCKR (0x0020 / sizeof(uint32_t))
#define AT91_PMC_MOR (0x30 / sizeof(uint32_t))
#define AT91_PMC_CSS             (0x3 <<  0) // (PMC) Programmable Clock Selection
#define AT91_PMC_MCKRDY          (0x1 <<  3) // (PMC) Master Clock Status/Enable/Disable/Mask
#define AT91_PMC_MOSCS           (0x1 <<  0) // (PMC) MOSC Status/Enable/Disable/Mask
#define AT91_CKGR_MOSCEN         (0x1 <<  0) // (CKGR) Main Oscillator Enable
#define AT91_PMC_PLLAR (0x0028 / sizeof(uint32_t))
#define AT91_PMC_PLLBR (0x002C / sizeof(uint32_t))
#define AT91_PMC_LOCKA           (0x1 <<  1) // (PMC) PLL A Status/Enable/Disable/Mask
#define AT91_PMC_LOCKB           (0x1 <<  2) // (PMC) PLL B Status/Enable/Disable/Mask
#define AT91_PMC_PCER (0x10 / sizeof(uint32_t))

/*dbgu registers */
#define AT91_DBGU_CR   0x0
#define AT91_DBGU_MR   (4 / sizeof(uint32_t))
#define AT91_DBGU_IER  (8 / sizeof(uint32_t))
#define AT91_DBGU_IDR  (0xC / sizeof(uint32_t))
#define AT91_DBGU_IMR  (0x10 / sizeof(uint32_t))
#define AT91_DBGU_SR   (0x14 / sizeof(uint32_t))
#define AT91_DBGU_RHR  (0x18 / sizeof(uint32_t))
#define AT91_DBGU_THR  (0x001C / sizeof(uint32_t))
#define AT91_DBGU_BRGR (0x0020 / sizeof(uint32_t))


// -------- DBGU_CR : (DBGU Offset: 0x0) Debug Unit Control Register --------
#define AT91_US_RSTRX            (0x1 <<  2) // (DBGU) Reset Receiver
#define AT91_US_RSTTX            (0x1 <<  3) // (DBGU) Reset Transmitter
#define AT91_US_RXEN             (0x1 <<  4) // (DBGU) Receiver Enable
#define AT91_US_RXDIS            (0x1 <<  5) // (DBGU) Receiver Disable
#define AT91_US_TXEN             (0x1 <<  6) // (DBGU) Transmitter Enable
#define AT91_US_TXDIS            (0x1 <<  7) // (DBGU) Transmitter Disable
#define AT91_US_RSTSTA           (0x1 <<  8) // (DBGU) Reset Status Bits
// -------- DBGU_IER : (DBGU Offset: 0x8) Debug Unit Interrupt Enable Register --------
#define AT91_US_RXRDY            (0x1 <<  0) // (DBGU) RXRDY Interrupt
#define AT91_US_TXRDY            (0x1 <<  1) // (DBGU) TXRDY Interrupt
#define AT91_US_ENDRX            (0x1 <<  3) // (DBGU) End of Receive Transfer Interrupt
#define AT91_US_ENDTX            (0x1 <<  4) // (DBGU) End of Transmit Interrupt
#define AT91_US_OVRE             (0x1 <<  5) // (DBGU) Overrun Interrupt
#define AT91_US_FRAME            (0x1 <<  6) // (DBGU) Framing Error Interrupt
#define AT91_US_PARE             (0x1 <<  7) // (DBGU) Parity Error Interrupt
#define AT91_US_TXEMPTY          (0x1 <<  9) // (DBGU) TXEMPTY Interrupt
#define AT91_US_TXBUFE           (0x1 << 11) // (DBGU) TXBUFE Interrupt
#define AT91_US_RXBUFF           (0x1 << 12) // (DBGU) RXBUFF Interrupt
#define AT91_US_COMM_TX          (0x1 << 30) // (DBGU) COMM_TX Interrupt
#define AT91_US_COMM_RX          (0x1 << 31) // (DBGU) COMM_RX Interrupt

/* US registers */
#define AT91_US_CR  (0)
#define AT91_US_MR  (4 / sizeof(uint32_t))
#define AT91_US_IER (8 / sizeof(uint32_t))
#define AT91_US_IDR (0xC / sizeof(uint32_t))
#define AT91_US_IMR (0x10 / sizeof(uint32_t))

/* matrix */

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR AHB Matrix Interface
// *****************************************************************************
// *** Register offset in AT91S_MATRIX structure ***
#define MATRIX_MCFG0    ( 0) //  Master Configuration Register 0
#define MATRIX_MCFG1    ( 4) //  Master Configuration Register 1
#define MATRIX_MCFG2    ( 8) //  Master Configuration Register 2
#define MATRIX_MCFG3    (12) //  Master Configuration Register 3
#define MATRIX_MCFG4    (16) //  Master Configuration Register 4
#define MATRIX_MCFG5    (20) //  Master Configuration Register 5
#define MATRIX_MCFG6    (24) //  Master Configuration Register 6
#define MATRIX_MCFG7    (28) //  Master Configuration Register 7
#define MATRIX_MCFG8    (32) //  Master Configuration Register 8
#define MATRIX_SCFG0    (64) //  Slave Configuration Register 0
#define MATRIX_SCFG1    (68) //  Slave Configuration Register 1
#define MATRIX_SCFG2    (72) //  Slave Configuration Register 2
#define MATRIX_SCFG3    (76) //  Slave Configuration Register 3
#define MATRIX_SCFG4    (80) //  Slave Configuration Register 4
#define MATRIX_SCFG5    (84) //  Slave Configuration Register 5
#define MATRIX_SCFG6    (88) //  Slave Configuration Register 6
#define MATRIX_SCFG7    (92) //  Slave Configuration Register 7
#define MATRIX_PRAS0    (128) //  PRAS0
#define MATRIX_PRBS0    (132) //  PRBS0
#define MATRIX_PRAS1    (136) //  PRAS1
#define MATRIX_PRBS1    (140) //  PRBS1
#define MATRIX_PRAS2    (144) //  PRAS2
#define MATRIX_PRBS2    (148) //  PRBS2
#define MATRIX_PRAS3    (152) //  PRAS3
#define MATRIX_PRBS3    (156) //  PRBS3
#define MATRIX_PRAS4    (160) //  PRAS4
#define MATRIX_PRBS4    (164) //  PRBS4
#define MATRIX_PRAS5    (168) //  PRAS5
#define MATRIX_PRBS5    (172) //  PRBS5
#define MATRIX_PRAS6    (176) //  PRAS6
#define MATRIX_PRBS6    (180) //  PRBS6
#define MATRIX_PRAS7    (184) //  PRAS7
#define MATRIX_PRBS7    (188) //  PRBS7
#define MATRIX_MRCR     (256) //  Master Remp Control Register

#define AT91C_MATRIX_RCA926I      (0x1 <<  0) // (MATRIX) Remap Command Bit for ARM926EJ-S Instruction
#define AT91C_MATRIX_RCA926D      (0x1 <<  1) // (MATRIX) Remap Command Bit for ARM926EJ-S Data
#define AT91C_MATRIX_RCB2         (0x1 <<  2) // (MATRIX) Remap Command Bit for PDC
#define AT91C_MATRIX_RCB3         (0x1 <<  3) // (MATRIX) Remap Command Bit for LCD
#define AT91C_MATRIX_RCB4         (0x1 <<  4) // (MATRIX) Remap Command Bit for 2DGC
#define AT91C_MATRIX_RCB5         (0x1 <<  5) // (MATRIX) Remap Command Bit for ISI
#define AT91C_MATRIX_RCB6         (0x1 <<  6) // (MATRIX) Remap Command Bit for DMA
#define AT91C_MATRIX_RCB7         (0x1 <<  7) // (MATRIX) Remap Command Bit for EMAC
#define AT91C_MATRIX_RCB8         (0x1 <<  8) // (MATRIX) Remap Command Bit for USB

/*pitc */
#define AT91_PTIC_MR_PITEN  (1 << 24)
#define AT91_PTIC_MR_PITIEN (1 << 25)
#define AT91_PITC_MR      0
#define AT91_PITC_SR   (0x4 / sizeof(uint32_t))
#define AT91_PITC_PIVR (0x8 / sizeof(uint32_t))
#define AT91_PITC_PIIR (0xC / sizeof(uint32_t))

/*AIC registers*/
#define AT91_AIC_SVR0  (0x80  / sizeof(uint32_t))
#define AT91_AIC_ISR   (0x108 / sizeof(uint32_t))
#define AT91_AIC_IECR  (0x120 / sizeof(uint32_t))
#define AT91_AIC_EOICR (0x130 / sizeof(uint32_t))
#define AT91_AIC_IVR   (0x100 / sizeof(uint32_t))
#define AT91_AIC_IDCR  (0x124 / sizeof(uint32_t))

#define AT91_PERIPH_SYS_ID 1

#endif//!_HW_AT91SAM9263_DEFS_H_
