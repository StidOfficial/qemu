#ifndef WPCM450_AIC_H
#define WPCM450_AIC_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define PXA2XX_PIC_SRCS	40

typedef struct WPCM450AICState {
    SysBusDevice parent;

    MemoryRegion iomem;
    qemu_irq parent_fiq;
    qemu_irq parent_irq;

    uint32_t int_enabled[2];
    uint32_t int_pending[2];
    uint32_t is_fiq[2];
    uint32_t int_idle;
    uint32_t priority[PXA2XX_PIC_SRCS];
} WPCM450AICState;

#define TYPE_WPCM450_AIC "wpcm450-aic"
#define WPCM450_AIC(obj) \
    OBJECT_CHECK(WPCM450AICState, (obj), TYPE_WPCM450_AIC)

#endif