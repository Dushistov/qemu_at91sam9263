#include <stdio.h>

#include "hw.h"
#include "at91.h"

typedef struct NandState {
    unsigned int cmd;
} NandState;

#define AT91_NAND_DEBUG
#ifdef AT91_NAND_DEBUG
#define DPRINTF(fmt, ...)                           \
    do {                                            \
        printf("AT91NAND: " fmt , ## __VA_ARGS__);    \
    } while (0)
#else
#define DPRINTF(fmt, ...) do { } while (0)
#endif

extern CPUState *g_env;

static uint32_t at91_nand_mem_read(void *opaque, target_phys_addr_t offset)
{
    NandState *s = opaque;

    DPRINTF("(IP %X) read from %X\n", g_env->regs[15], offset);
    switch (s->cmd) {
    case 0x70:
        s->cmd = 0;
        return 0x40;
    default:
        return 0x0;
    }
}

static void at91_nand_mem_write(void *opaque, target_phys_addr_t offset,
                uint32_t value)
{
    NandState *s = opaque;

    DPRINTF("(IP %X) write to %X %X\n", g_env->regs[15], offset, value);
    s->cmd = value;
}

static CPUReadMemoryFunc *at91_nand_readfn[] = {
    at91_nand_mem_read,
    at91_nand_mem_read,
    at91_nand_mem_read,
};

static CPUWriteMemoryFunc *at91_nand_writefn[] = {
    at91_nand_mem_write,
    at91_nand_mem_write,
    at91_nand_mem_write,
};

void at91_nand_register(void)
{
    NandState *s;
    int iomemtype;

    s = qemu_mallocz(sizeof(*s));
    iomemtype = cpu_register_io_memory(at91_nand_readfn, at91_nand_writefn, s);
    cpu_register_physical_memory(0x40000000, 0x10000000, iomemtype);
}


