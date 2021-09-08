#ifndef WPCM450_SMC_H
#define WPCM450_SMC_H

#include "hw/sysbus.h"
#include "qom/object.h"

typedef struct WPCM450SMCState {
    SysBusDevice parent;

    MemoryRegion iomem;
} WPCM450SMCState;

#define TYPE_WPCM450_SMC "wpcm450-smc"
#define WPCM450_SMC(obj) \
    OBJECT_CHECK(WPCM450SMCState, (obj), TYPE_WPCM450_SMC)

#endif