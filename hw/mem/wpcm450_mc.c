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

#include "hw/mem/wpcm450_mc.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/log.h"

enum WPCM450MCRegisters {
    WPCM450_MC_CFG0,
    WPCM450_MC_CFG1,
    WPCM450_MC_CFG2,
    WPCM450_MC_CFG3,
    WPCM450_MC_CFG4,
    WPCM450_MC_CFG5,
    WPCM450_MC_CFG6,
    WPCM450_MC_CFG7,
    WPCM450_MC_P1_ARBT,
    WPCM450_MC_P1_CNT,
    WPCM450_MC_P2_ARBT,
    WPCM450_MC_P2_CNT,
    WPCM450_MC_P3_ARBT,
    WPCM450_MC_P3_CNT,
    WPCM450_MC_P4_ARBT,
    WPCM450_MC_P4_CNT,
    WPCM450_MC_P5_ARBT,
    WPCM450_MC_P5_CNT,
    WPCM450_MC_P6_ARBT,
    WPCM450_MC_P6_CNT,
    WPCM450_MC_P1_INCRS,
    WPCM450_MC_P2_INCRS,
    WPCM450_MC_P3_INCRS,
    WPCM450_MC_P4_INCRS,
    WPCM450_MC_DLL_0,
    WPCM450_MC_DLL_1,
};

static uint64_t wpcm450_mc_mem_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    WPCM450MCState *s = opaque;
    uint32_t reg = offset / sizeof(uint32_t);

    if(reg >= WPCM450_MC_NR_REGS) {
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

static void wpcm450_mc_mem_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
    WPCM450MCState *s = opaque;
    uint32_t reg = offset / sizeof(uint32_t);

    if(reg >= WPCM450_MC_NR_REGS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                        "%s: offset 0x%04" HWADDR_PRIx " out of range\n",
                        __func__, offset);
        return;
    }

    qemu_log_mask(LOG_UNIMP,
                    "%s: register @ 0x%04" HWADDR_PRIx " is unimplemented\n",
                    __func__, offset);

    s->regs[reg] = value;
}

static const MemoryRegionOps wpcm450_mc_ops = {
    .read = wpcm450_mc_mem_read,
    .write = wpcm450_mc_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void wpcm450_mc_init(Object *obj)
{
    WPCM450MCState *s = WPCM450_MC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    //s->regs[WPCM450_GCR_PDID] = PDID_A3;

    /* Enable IC memory-mapped registers access.  */
    memory_region_init_io(&s->iomem, OBJECT(s), &wpcm450_mc_ops, s,
                          TYPE_WPCM450_MC, 0x68);
    sysbus_init_mmio(sbd, &s->iomem);
}

static VMStateDescription vmstate_wpcm450_mc_regs = {
    .name = "wpcm450-mc",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, WPCM450MCState, WPCM450_MC_NR_REGS),
        VMSTATE_END_OF_LIST(),
    },
};

static void wpcm450_mc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "WPCM450 Memory Controller";
    dc->vmsd = &vmstate_wpcm450_mc_regs;
}

static const TypeInfo wpcm450_mc_info = {
    .name          = TYPE_WPCM450_MC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(WPCM450MCState),
    .class_init    = wpcm450_mc_class_init,
    .instance_init = wpcm450_mc_init,
};

static void wpcm450_mc_register_types(void)
{
    type_register_static(&wpcm450_mc_info);
}

type_init(wpcm450_mc_register_types)