/*
 * WPCM450 Global Control Registers.
 *
 * Copyright (c) 2006 Openedhand Ltd.
 * Copyright (c) 2006 Thorsten Zitterell
 * Written by Andrzej Zaborowski <balrog@zabor.org>
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"

#include "hw/misc/wpcm450_gcr.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/log.h"

/* PDID - product identifier register fields */
#define PDID_Z1     0x00926450
#define PDID_Z2     0x03926450
#define PDID_Z21    0x04926450
#define PDID_A1     0x08926450
#define PDID_A2     0x09926450
#define PDID_A3     0x0A926450

enum WPCM450GCRRegisters {
    WPCM450_GCR_PDID,
    WPCM450_GCR_PWRON,
    WPCM450_GCR_MFSEL1,
    WPCM450_GCR_MFSEL2,
    WPCM450_GCR_MISCPE,
    WPCM450_GCR_GPIO1PE,
    WPCM450_GCR_GPIO2PE,
    WPCM450_GCR_GPIO3PE,
    WPCM450_GCR_GPIO5PE,
    WPCM450_GCR_GPIO6PE,
    WPCM450_GCR_GPIO7PE,
    WPCM450_GCR_SPSWC,
    WPCM450_GCR_INTCR,
    WPCM450_GCR_XBCR,
    WPCM450_GCR_HIFCR,
    WPCM450_GCR_INTCR2,
    WPCM450_GCR_ETSR,
};

static uint64_t wpcm450_gcr_mem_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    WPCM450GCRState *s = opaque;
    uint32_t reg = offset / sizeof(uint32_t);

    if(reg >= WPCM450_GCR_NR_REGS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                        "%s: offset 0x%04" HWADDR_PRIx " out of range\n",
                        __func__, offset);
        return 0;
    }

    return s->regs[reg];
}

static void wpcm450_gcr_mem_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
    WPCM450GCRState *s = opaque;
    uint32_t reg = offset / sizeof(uint32_t);

    if(reg >= WPCM450_GCR_NR_REGS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                        "%s: offset 0x%04" HWADDR_PRIx " out of range\n",
                        __func__, offset);
        return;
    }

    switch(reg) {
        case WPCM450_GCR_PDID:
        case WPCM450_GCR_PWRON:
            qemu_log_mask(LOG_GUEST_ERROR,
                            "%s: register @ 0x%04" HWADDR_PRIx " is read-only\n",
                            __func__, offset);
            return;
    }

    s->regs[reg] = value;
}

static const MemoryRegionOps wpcm450_gcr_ops = {
    .read = wpcm450_gcr_mem_read,
    .write = wpcm450_gcr_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void wpcm450_gcr_init(Object *obj)
{
    WPCM450GCRState *s = WPCM450_GCR(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->regs[WPCM450_GCR_PDID] = PDID_A3;

    /* Enable IC memory-mapped registers access.  */
    memory_region_init_io(&s->iomem, OBJECT(s), &wpcm450_gcr_ops, s,
                          TYPE_WPCM450_GCR, 0x200);
    sysbus_init_mmio(sbd, &s->iomem);
}

static VMStateDescription vmstate_wpcm450_gcr_regs = {
    .name = "wpcm450-gcr",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, WPCM450GCRState, WPCM450_GCR_NR_REGS),
        VMSTATE_END_OF_LIST(),
    },
};

static void wpcm450_gcr_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "WPCM450 Global Control Registers";
    dc->vmsd = &vmstate_wpcm450_gcr_regs;
}

static const TypeInfo wpcm450_gcr_info = {
    .name          = TYPE_WPCM450_GCR,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(WPCM450GCRState),
    .class_init    = wpcm450_gcr_class_init,
    .instance_init = wpcm450_gcr_init,
};

static void wpcm450_gcr_register_types(void)
{
    type_register_static(&wpcm450_gcr_info);
}

type_init(wpcm450_gcr_register_types)