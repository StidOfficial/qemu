/*
 * WPCM450 General Purpose Input/Output Register.
 *
 * Copyright (c) 2006 Openedhand Ltd.
 * Copyright (c) 2006 Thorsten Zitterell
 * Written by Andrzej Zaborowski <balrog@zabor.org>
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"

#include "hw/gpio/wpcm450_gpio.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/log.h"

static uint64_t wpcm450_gpio_mem_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    WPCM450GPIOState *s = opaque;
    uint32_t reg = offset / sizeof(uint32_t);

    if(reg >= WPCM450_GPIO_NR_REGS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                        "%s: offset 0x%04" HWADDR_PRIx " out of range\n",
                        __func__, offset);
        return 0;
    }

    qemu_log_mask(LOG_UNIMP,
                    "%s: register @ 0x%04" HWADDR_PRIx " is unimplemented\n",
                    __func__, offset);

    return s->regs[reg];
}

static void wpcm450_gpio_mem_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
    WPCM450GPIOState *s = opaque;
    uint32_t reg = offset / sizeof(uint32_t);

    if(reg >= WPCM450_GPIO_NR_REGS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                        "%s: offset 0x%04" HWADDR_PRIx " out of range\n",
                        __func__, offset);
        return;
    }

    qemu_log_mask(LOG_UNIMP,
                    "%s: register @ 0x%04" HWADDR_PRIx " is unimplemented\n",
                    __func__, offset);

    /*switch(reg) {
        case WPCM450_GCR_PDID:
        case WPCM450_GCR_PWRON:
            qemu_log_mask(LOG_GUEST_ERROR,
                            "%s: register @ 0x%04" HWADDR_PRIx " is read-only\n",
                            __func__, offset);
            return;
    }*/

    s->regs[reg] = value;
}

static const MemoryRegionOps wpcm450_gpio_ops = {
    .read = wpcm450_gpio_mem_read,
    .write = wpcm450_gpio_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void wpcm450_gpio_init(Object *obj)
{
    WPCM450GPIOState *s = WPCM450_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    /* Enable IC memory-mapped registers access.  */
    memory_region_init_io(&s->iomem, OBJECT(s), &wpcm450_gpio_ops, s,
                          TYPE_WPCM450_GPIO, 0xa4);
    sysbus_init_mmio(sbd, &s->iomem);
}

static VMStateDescription vmstate_wpcm450_gpio_regs = {
    .name = "wpcm450-gpio",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, WPCM450GPIOState, WPCM450_GPIO_NR_REGS),
        VMSTATE_END_OF_LIST(),
    },
};

static void wpcm450_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "WPCM450 Global Control Registers";
    dc->vmsd = &vmstate_wpcm450_gpio_regs;
}

static const TypeInfo wpcm450_gpio_info = {
    .name          = TYPE_WPCM450_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(WPCM450GPIOState),
    .class_init    = wpcm450_gpio_class_init,
    .instance_init = wpcm450_gpio_init,
};

static void wpcm450_gpio_register_types(void)
{
    type_register_static(&wpcm450_gpio_info);
}

type_init(wpcm450_gpio_register_types)