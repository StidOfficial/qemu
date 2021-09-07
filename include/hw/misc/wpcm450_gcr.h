#ifndef WPCM450_GCR_H
#define WPCM450_GCR_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define WPCM450_GCR_NR_REGS (0x200 / sizeof(uint32_t))

typedef struct WPCM450GCRState {
    SysBusDevice parent;

    MemoryRegion iomem;
    
    uint32_t regs[WPCM450_GCR_NR_REGS];
} WPCM450GCRState;

#define TYPE_WPCM450_GCR "wpcm450-gcr"
#define WPCM450_GCR(obj) \
    OBJECT_CHECK(WPCM450GCRState, (obj), TYPE_WPCM450_GCR)

#endif