#include <stdio.h>

#include "hw.h"
#include "flash.h"
#include "at91.h"

typedef struct NandState {
    NANDFlashState *nand_state;
} NandState;

//#define AT91_NAND_DEBUG
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

    uint8_t res = nand_getio(s->nand_state);
    DPRINTF("(IP %X) read from %X (res %X)\n", g_env->regs[15], offset, res);
    return res;
}

static void at91_nand_mem_write(void *opaque, target_phys_addr_t offset,
                uint32_t value)
{
    NandState *s = opaque;        

    nand_setpins(s->nand_state, offset & (1 << 22), offset & (1 << 21), 0, 1, 0);
    DPRINTF("(IP %X) write to %X %X\n", g_env->regs[15], offset, value);
    nand_setio(s->nand_state, value & 0xFF);
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

void at91_nand_register(NANDFlashState *st)
{
    NandState *s;
    int iomemtype;

    s = qemu_mallocz(sizeof(*s));
    s->nand_state = st;
    iomemtype = cpu_register_io_memory(at91_nand_readfn, at91_nand_writefn, s);
    cpu_register_physical_memory(0x40000000, 0x10000000, iomemtype);
}


