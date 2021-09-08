#ifndef WPCM450_CPLD_H
#define WPCM450_CPLD_H

#include "hw/sysbus.h"
#include "qom/object.h"

typedef struct WPCM450CPLDState {
    SysBusDevice parent;

    MemoryRegion iomem;
} WPCM450CPLDState;

#define TYPE_WPCM450_CPLD "wpcm450-cpld"
#define WPCM450_CPLD(obj) \
    OBJECT_CHECK(WPCM450CPLDState, (obj), TYPE_WPCM450_CPLD)

#endif