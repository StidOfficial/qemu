/*
 * WPCM450 Advanced Interrupt Controller.
 *
 * Copyright (c) 2006 Openedhand Ltd.
 * Copyright (c) 2006 Thorsten Zitterell
 * Written by Andrzej Zaborowski <balrog@zabor.org>
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/log.h"
#include "hw/arm/pxa.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "hw/intc/wpcm450_aic.h"

#define ICIP	0x00	/* Interrupt Controller IRQ Pending register */
#define ICMR	0x04	/* Interrupt Controller Mask register */
#define ICLR	0x08	/* Interrupt Controller Level register */
#define ICFP	0x0c	/* Interrupt Controller FIQ Pending register */
#define ICPR	0x10	/* Interrupt Controller Pending register */
#define ICCR	0x14	/* Interrupt Controller Control register */
#define ICHP	0x18	/* Interrupt Controller Highest Priority register */
#define IPR0	0x1c	/* Interrupt Controller Priority register 0 */
#define IPR31	0x98	/* Interrupt Controller Priority register 31 */
#define ICIP2	0x9c	/* Interrupt Controller IRQ Pending register 2 */
#define ICMR2	0xa0	/* Interrupt Controller Mask register 2 */
#define ICLR2	0xa4	/* Interrupt Controller Level register 2 */
#define ICFP2	0xa8	/* Interrupt Controller FIQ Pending register 2 */
#define ICPR2	0xac	/* Interrupt Controller Pending register 2 */
#define IPR32	0xb0	/* Interrupt Controller Priority register 32 */
#define IPR39	0xcc	/* Interrupt Controller Priority register 39 */

static void wpcm450_aic_update(WPCM450AICState *s)
{
    /*qemu_set_irq(s->parent_fiq, irq);
    qemu_set_irq(s->parent_fiq, irq);*/
}

/* Note: Here level means state of the signal on a pin, not
 * IRQ/FIQ distinction as in PXA Developer Manual.  */
static void wpcm450_aic_set_irq(void *opaque, int irq, int level)
{
    WPCM450AICState *s = (WPCM450AICState *) opaque;
#ifdef IGNORE
    int int_set = (irq >= 32);
    irq &= 31;

    if (level)
        s->int_pending[int_set] |= 1 << irq;
    else
        s->int_pending[int_set] &= ~(1 << irq);

    pxa2xx_pic_update(opaque);
#endif

    wpcm450_aic_update(s);
}

#ifdef IGNORE
static inline uint32_t pxa2xx_pic_highest(WPCM450AICState *s) {
    int i, int_set, irq;
    uint32_t bit, mask[2];
    uint32_t ichp = 0x003f003f;	/* Both IDs invalid */

    mask[0] = s->int_pending[0] & s->int_enabled[0];
    mask[1] = s->int_pending[1] & s->int_enabled[1];

    for (i = PXA2XX_PIC_SRCS - 1; i >= 0; i --) {
        irq = s->priority[i] & 0x3f;
        if ((s->priority[i] & (1U << 31)) && irq < PXA2XX_PIC_SRCS) {
            /* Source peripheral ID is valid.  */
            bit = 1 << (irq & 31);
            int_set = (irq >= 32);

            if (mask[int_set] & bit & s->is_fiq[int_set]) {
                /* FIQ asserted */
                ichp &= 0xffff0000;
                ichp |= (1 << 15) | irq;
            }

            if (mask[int_set] & bit & ~s->is_fiq[int_set]) {
                /* IRQ asserted */
                ichp &= 0x0000ffff;
                ichp |= (1U << 31) | (irq << 16);
            }
        }
    }

    return ichp;
}
#endif

static uint64_t wpcm450_aic_mem_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
#ifdef IGNORE
    WPCM450AICState *s = (WPCM450AICState *) opaque;

    switch (offset) {
    case ICIP:	/* IRQ Pending register */
        return s->int_pending[0] & ~s->is_fiq[0] & s->int_enabled[0];
    case ICIP2:	/* IRQ Pending register 2 */
        return s->int_pending[1] & ~s->is_fiq[1] & s->int_enabled[1];
    case ICMR:	/* Mask register */
        return s->int_enabled[0];
    case ICMR2:	/* Mask register 2 */
        return s->int_enabled[1];
    case ICLR:	/* Level register */
        return s->is_fiq[0];
    case ICLR2:	/* Level register 2 */
        return s->is_fiq[1];
    case ICCR:	/* Idle mask */
        return (s->int_idle == 0);
    case ICFP:	/* FIQ Pending register */
        return s->int_pending[0] & s->is_fiq[0] & s->int_enabled[0];
    case ICFP2:	/* FIQ Pending register 2 */
        return s->int_pending[1] & s->is_fiq[1] & s->int_enabled[1];
    case ICPR:	/* Pending register */
        return s->int_pending[0];
    case ICPR2:	/* Pending register 2 */
        return s->int_pending[1];
    case IPR0  ... IPR31:
        return s->priority[0  + ((offset - IPR0 ) >> 2)];
    case IPR32 ... IPR39:
        return s->priority[32 + ((offset - IPR32) >> 2)];
    case ICHP:	/* Highest Priority register */
        return pxa2xx_pic_highest(s);
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "wpcm450_aic_mem_read: bad register offset 0x%" HWADDR_PRIx
                      "\n", offset);
        return 0;
    }
#endif
    qemu_log_mask(LOG_UNIMP,
                    "%s: register @ 0x%04" HWADDR_PRIx " is unimplemented\n",
                    __func__, offset);

    return 0;
}

static void wpcm450_aic_mem_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
    WPCM450AICState *s = (WPCM450AICState *) opaque;

#ifdef IGNORE
    switch (offset) {
    case ICMR:	/* Mask register */
        s->int_enabled[0] = value;
        break;
    case ICMR2:	/* Mask register 2 */
        s->int_enabled[1] = value;
        break;
    case ICLR:	/* Level register */
        s->is_fiq[0] = value;
        break;
    case ICLR2:	/* Level register 2 */
        s->is_fiq[1] = value;
        break;
    case ICCR:	/* Idle mask */
        s->int_idle = (value & 1) ? 0 : ~0;
        break;
    case IPR0  ... IPR31:
        s->priority[0  + ((offset - IPR0 ) >> 2)] = value & 0x8000003f;
        break;
    case IPR32 ... IPR39:
        s->priority[32 + ((offset - IPR32) >> 2)] = value & 0x8000003f;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "wpcm450_aic_mem_write: bad register offset 0x%"
                      HWADDR_PRIx "\n", offset);
        return;
    }
#endif

    qemu_log_mask(LOG_UNIMP,
                    "%s: register @ 0x%04" HWADDR_PRIx " is unimplemented\n",
                    __func__, offset);

    wpcm450_aic_update(s);
}

static const MemoryRegionOps wpcm450_aic_ops = {
    .read = wpcm450_aic_mem_read,
    .write = wpcm450_aic_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void wpcm450_aic_init(Object *obj)
{
    WPCM450AICState *s = WPCM450_AIC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(obj);

    s->int_pending[0] = 0;
    s->int_pending[1] = 0;
    s->int_enabled[0] = 0;
    s->int_enabled[1] = 0;
    s->is_fiq[0] = 0;
    s->is_fiq[1] = 0;

    qdev_init_gpio_in(dev, wpcm450_aic_set_irq, PXA2XX_PIC_SRCS);

    /* Enable IC memory-mapped registers access.  */
    memory_region_init_io(&s->iomem, OBJECT(s), &wpcm450_aic_ops, s,
                          TYPE_WPCM450_AIC, 0x134);
    sysbus_init_mmio(sbd, &s->iomem);
}

static VMStateDescription vmstate_wpcm450_aic_regs = {
    .name = "wpcm450-aic",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(int_enabled, WPCM450AICState, 2),
        VMSTATE_UINT32_ARRAY(int_pending, WPCM450AICState, 2),
        VMSTATE_UINT32_ARRAY(is_fiq, WPCM450AICState, 2),
        VMSTATE_UINT32(int_idle, WPCM450AICState),
        VMSTATE_UINT32_ARRAY(priority, WPCM450AICState, PXA2XX_PIC_SRCS),
        VMSTATE_END_OF_LIST(),
    },
};

static void wpcm450_aic_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "WPCM450 AIC";
    dc->vmsd = &vmstate_wpcm450_aic_regs;
}

static const TypeInfo wpcm450_aic_info = {
    .name          = TYPE_WPCM450_AIC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(WPCM450AICState),
    .class_init    = wpcm450_aic_class_init,
    .instance_init = wpcm450_aic_init,
};

static void wpcm450_aic_register_types(void)
{
    type_register_static(&wpcm450_aic_info);
}

type_init(wpcm450_aic_register_types)