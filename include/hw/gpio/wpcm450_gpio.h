#ifndef WPCM450_GPIO_H
#define WPCM450_GPIO_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define WPCM450_GPIO_NR_REGS (0xa4 / sizeof(uint32_t))

typedef struct WPCM450GPIOState {
    SysBusDevice parent;

    MemoryRegion iomem;

    uint32_t regs[WPCM450_GPIO_NR_REGS];
} WPCM450GPIOState;

#define TYPE_WPCM450_GPIO "wpcm450-gpio"
#define WPCM450_GPIO(obj) \
    OBJECT_CHECK(WPCM450GPIOState, (obj), TYPE_WPCM450_GPIO)

#endif