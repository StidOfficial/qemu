/*
 * WPCM450 Shared Memory Core Control Register (SMC).
 *
 * Copyright (c) 2006 Openedhand Ltd.
 * Copyright (c) 2006 Thorsten Zitterell
 * Written by Andrzej Zaborowski <balrog@zabor.org>
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"

#include "hw/mem/wpcm450_smc.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/log.h"

static uint64_t wpcm450_smc_mem_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    qemu_log_mask(LOG_UNIMP,
                    "%s: register @ 0x%04" HWADDR_PRIx " is unimplemented\n",
                    __func__, offset);

    return 0;
}

static void wpcm450_smc_mem_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
    qemu_log_mask(LOG_UNIMP,
                    "%s: register @ 0x%04" HWADDR_PRIx " is unimplemented\n",
                    __func__, offset);
}

static const MemoryRegionOps wpcm450_smc_ops = {
    .read = wpcm450_smc_mem_read,
    .write = wpcm450_smc_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void wpcm450_smc_init(Object *obj)
{
    WPCM450SMCState *s = WPCM450_SMC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    /* Enable IC memory-mapped registers access.  */
    memory_region_init_io(&s->iomem, OBJECT(s), &wpcm450_smc_ops, s,
                          TYPE_WPCM450_SMC, 0x1);
    sysbus_init_mmio(sbd, &s->iomem);
}

static VMStateDescription vmstate_wpcm450_smc_regs = {
    .name = "wpcm450-smc",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST(),
    },
};

static void wpcm450_smc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "WPCM450 Shared Memory Core Control Register";
    dc->vmsd = &vmstate_wpcm450_smc_regs;
}

static const TypeInfo wpcm450_smc_info = {
    .name          = TYPE_WPCM450_SMC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(WPCM450SMCState),
    .class_init    = wpcm450_smc_class_init,
    .instance_init = wpcm450_smc_init,
};

static void wpcm450_smc_register_types(void)
{
    type_register_static(&wpcm450_smc_info);
}

type_init(wpcm450_smc_register_types)