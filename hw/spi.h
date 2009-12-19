#ifndef _HW_SPI_H_
#define _HW_SPI_H_

/* minimal interface to separate device connected via SPI and SPI controller */
typedef struct SPIControl {
    void *opaque;
    uint32_t (*txrx_callback)(void *opaque, uint32_t val, int len);
    void (*set_chipselect)(void *opaque, int on);
} SPIControl;

#endif//!_HW_SPI_H_
