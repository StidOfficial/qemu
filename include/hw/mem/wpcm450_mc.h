#ifndef WPCM450_MC_H
#define WPCM450_MC_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define WPCM450_MC_NR_REGS (0x68 / sizeof(uint32_t))

typedef struct WPCM450MCState {
    SysBusDevice parent;

    MemoryRegion iomem;
    
    uint32_t regs[WPCM450_MC_NR_REGS];
} WPCM450MCState;

#define TYPE_WPCM450_MC "wpcm450-mc"
#define WPCM450_MC(obj) \
    OBJECT_CHECK(WPCM450MCState, (obj), TYPE_WPCM450_MC)

#endif