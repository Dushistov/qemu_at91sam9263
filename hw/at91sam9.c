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
#include "spi.h"
#include "flash.h"
#include "boards.h"
#include "qemu-char.h"
#include "qemu-timer.h"
#include "sysbus.h"
#include "at91.h"

#include "at91sam9263_defs.h"

//#define AT91_TRACE_ON
#define AT91_DEBUG_ON

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
    uint32_t bus_matrix_regs[0x100 / 4];
    uint32_t ccfg_regs[(0x01FC - 0x0110 + 1) / sizeof(uint32_t)];
    uint32_t sdramc0_regs[(AT91_SMC0_BASE - AT91_SDRAMC0_BASE) / sizeof(uint32_t)];
    uint32_t smc0_regs[(AT91_ECC1_BASE - AT91_SMC0_BASE) / sizeof(uint32_t)];
    uint32_t usart0_regs[0x1000 / sizeof(uint32_t)];
//    struct dbgu_state dbgu;
    pflash_t *norflash;
    ram_addr_t internal_sram;
    QEMUTimer *dbgu_tr_timer;
    ptimer_state *pitc_timer;
    int timer_active;
    CPUState *env;
    qemu_irq *qirq;
    ram_addr_t bootrom;
    int rom_size;
};

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

static void at91_init_bus_matrix(struct at91sam9_state *sam9)
{
    DEBUG("begin\n");
    memset(&sam9->bus_matrix_regs, 0, sizeof(sam9->bus_matrix_regs));
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

CPUState *g_env;

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
    qemu_irq *cpu_pic;
    qemu_irq pic[32];
    qemu_irq pic1[32];
    DeviceState *dev;
    DeviceState *pit;
    DeviceState *pmc;
    DeviceState *spi;
    int i;
    int bms;
    SPIControl *cs0_spi_handler;

    cs0_spi_handler = qemu_mallocz(sizeof(SPIControl));
    DEBUG("begin, ram_size %llu, boot dev %s\n", (unsigned long long)ram_size,
          boot_device ? boot_device : "<empty>");

    if (option_rom[0] && boot_device[0] == 'n') {
        printf("Emulate ROM code\n");
        bms = 1;
    } else {
        printf("Emulate start from EBI0_NCS0\n");
        bms = 0;
    }

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
    sam9->bootrom = qemu_ram_alloc(0x100000);
    cpu_register_physical_memory(0x00400000, 0x100000, sam9->bootrom | IO_MEM_RAM);
    if (option_rom[0]) {
        sam9->rom_size = load_image_targphys(option_rom[0], 0x00400000, 0x100000);
        printf("load bootrom, size %d\n", sam9->rom_size);
    }

    /*Internal Peripherals */
    iomemtype = cpu_register_io_memory(at91_periph_readfn, at91_periph_writefn, sam9);
    cpu_register_physical_memory(0xF0000000, 0xFFFFFFFF - 0xF0000000, iomemtype);

    cpu_pic = arm_pic_init_cpu(env);
    dev = sysbus_create_varargs("at91,aic", AT91_AIC_BASE,
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
    sysbus_create_simple("at91,dbgu", AT91_DBGU_BASE, pic1[0]);
    pmc = sysbus_create_simple("at91,pmc", AT91_PMC_BASE, pic1[1]);
    qdev_prop_set_uint32(pmc, "mo_freq", 16000000);
    pit = sysbus_create_simple("at91,pit", AT91_PITC_BASE, pic1[3]);
    sysbus_create_varargs("at91,tc", AT91_TC012_BASE, pic[19], pic[19], pic[19], NULL);
    spi = sysbus_create_simple("at91,spi", AT91_SPI0_BASE, pic[14]);
    at91_init_bus_matrix(sam9);
    memset(&sam9->ccfg_regs, 0, sizeof(sam9->ccfg_regs));
    sysbus_create_simple("at91,pio", AT91_PIOA_BASE, pic[2]);
    sysbus_create_simple("at91,pio", AT91_PIOB_BASE, pic[3]);
    sysbus_create_simple("at91,pio", AT91_PIOC_BASE, pic[4]);
    sysbus_create_simple("at91,pio", AT91_PIOD_BASE, pic[4]);
    sysbus_create_simple("at91,pio", AT91_PIOE_BASE, pic[4]);
    sysbus_create_varargs("at91,rstc", AT91_RSTC_BASE, NULL);
    memset(&sam9->sdramc0_regs, 0, sizeof(sam9->sdramc0_regs));
    memset(&sam9->smc0_regs, 0, sizeof(sam9->smc0_regs));
    memset(sam9->usart0_regs, 0, sizeof(sam9->usart0_regs));

    qemu_check_nic_model(&nd_table[0], "at91");
    dev = qdev_create(NULL, "at91,emac");
    dev->nd = &nd_table[0];
    qdev_init(dev);
    sysbus_mmio_map(sysbus_from_qdev(dev), 0, AT91_EMAC_BASE);
    sysbus_connect_irq(sysbus_from_qdev(dev), 0, pic[21]);

    sysbus_create_simple("at91,lcdc", AT91_LCDC_BASE, pic[26]);
    /*
      we use variant of booting from external memory (NOR FLASH),
      it mapped to 0x0 at start, and also it is accessable from 0x10000000 address
    */
    dinfo = drive_get(IF_PFLASH, 0, 0);
    if (dinfo) {
        if (bms) {
            NANDFlashState *nand_state;

            if (spi_flash_register(dinfo->bdrv, 4 * 1024 * 1024, cs0_spi_handler) < 0) {
                fprintf(stderr, "init of spi flash failed\n");
                exit(EXIT_FAILURE);
            }
            qdev_prop_set_ptr(spi, "spi_control", cs0_spi_handler);
            //rom
            cpu_register_physical_memory(0x0, 100 * 1024,
                                         sam9->bootrom | IO_MEM_ROMD);
            nand_state = nand_init(NAND_MFR_MICRON, 0xba);
            at91_nand_register(nand_state);
        } else {
            //nor flash
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
            cpu_register_physical_memory(0x0, 100 * 1024,
                                         nor_flash_mem | IO_MEM_ROMD);
        }
    } else {
        fprintf(stderr, "qemu: can not start without flash.\n");
        exit(EXIT_FAILURE);
    }
    g_env = env;
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
