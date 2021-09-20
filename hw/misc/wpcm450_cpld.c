/*
 * WPCM450 Complex Programmable Logic Devices.
 *
 * Copyright (c) 2006 Openedhand Ltd.
 * Copyright (c) 2006 Thorsten Zitterell
 * Written by Andrzej Zaborowski <balrog@zabor.org>
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"

#include "hw/misc/wpcm450_cpld.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/log.h"

/*
0x3 - bit 0-3 (platform id)

0x12 - bit 1 (AMEA is present)

0x21 - bit 0-2 -> 3 (Serial MUX)

#define CPLD_MUX_MODE_1     3 (011)
#define CPLD_MUX_MODE_2A    0 (000)
#define CPLD_MUX_MODE_2A_S  4 (100)
#define CPLD_MUX_MODE_2B    1 (001)
#define CPLD_MUX_MODE_2C    2 (010)

0x22 - bit 5 (extended platform id)

0x27 - bit 1 (System power on)
0 = Enable
1 = Disable
*/

enum WPCM450PlatformID {
    WPCM450_PLATFORM_ID_BLUEFISH,
    WPCM450_PLATFORM_ID_THIDWICK,
    WPCM450_PLATFORM_ID_SNEETCH,
    WPCM450_PLATFORM_ID_MACK,

    // Blade
    WPCM450_PLATFORM_ID_BARBALOOT,
    WPCM450_PLATFORM_ID_ZOOKS,
    WPCM450_PLATFORM_ID_MCCAVE,

    // MASER Lite
    WPCM450_PLATFORM_ID_YERTLE,
    WPCM450_PLATFORM_ID_CINDYLOU,
    WPCM450_PLATFORM_ID_MCBEAN,
    WPCM450_PLATFORM_ID_MAYZIE,
    WPCM450_PLATFORM_ID_SAMIAM,
    WPCM450_PLATFORM_ID_HORTON,
    
    WPCM450_PLATFORM_ID_UNSED_0,
    WPCM450_PLATFORM_ID_UNSED_1,

    WPCM450_PLATFORM_ID_BRUTUS,
    WPCM450_PLATFORM_ID_CLOVER,

    WPCM450_PLATFORM_ID_UNSED_2,

    WPCM450_PLATFORM_ID_SKIPPER,
    WPCM450_PLATFORM_ID_SLINKY,
    WPCM450_PLATFORM_ID_DIAMAS,
    WPCM450_PLATFORM_ID_COASTER,

    WPCM450_PLATFORM_ID_UNKNOWN = 0xff,
};

#define PLATFORM_ID             0x3
#define AMEA                    0x12
#define EXTENDED_PLATFORM_ID    0x22

static uint64_t wpcm450_cpld_mem_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    uint8_t temp;
    uint8_t byte = 0xff;

    uint8_t platform_id = WPCM450_PLATFORM_ID_THIDWICK;
    bool amea_present = false;

    switch(offset) {
        case PLATFORM_ID:
            for(int i = 0; i < 4; i++) {
                temp = ((platform_id >> i) & 1) << i;
                byte &= ~(1 << i);
                byte |= temp;
            }

            return byte; // bit 0-3
        case AMEA:
            temp = ((!amea_present >> 0) & 1) << 1;
            byte &= ~(1 << 1);
            byte |= temp;

            return byte; // bit 1; 0 = Present, 1 = Non present
        case EXTENDED_PLATFORM_ID:
            temp = ((platform_id >> 4) & 1) << 2;
            byte &= ~(1 << 2);
            byte |= temp;

            return byte; // bit 1
        default:
            qemu_log_mask(LOG_UNIMP,
                    "%s: register @ 0x%04" HWADDR_PRIx " is unimplemented\n",
                    __func__, offset);
    }

    return 0;
}

static void wpcm450_cpld_mem_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
    qemu_log_mask(LOG_UNIMP,
                    "%s: register @ 0x%04" HWADDR_PRIx " is unimplemented\n",
                    __func__, offset);
}

static const MemoryRegionOps wpcm450_cpld_ops = {
    .read = wpcm450_cpld_mem_read,
    .write = wpcm450_cpld_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void wpcm450_cpld_init(Object *obj)
{
    WPCM450CPLDState *s = WPCM450_CPLD(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    /* Enable IC memory-mapped registers access.  */
    memory_region_init_io(&s->iomem, OBJECT(s), &wpcm450_cpld_ops, s,
                          TYPE_WPCM450_CPLD, 0x30);
    sysbus_init_mmio(sbd, &s->iomem);
}

static VMStateDescription vmstate_wpcm450_cpld_regs = {
    .name = "wpcm450-cpld",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST(),
    },
};

static void wpcm450_cpld_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "WPCM450 CPLD";
    dc->vmsd = &vmstate_wpcm450_cpld_regs;
}

static const TypeInfo wpcm450_cpld_info = {
    .name          = TYPE_WPCM450_CPLD,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(WPCM450CPLDState),
    .class_init    = wpcm450_cpld_class_init,
    .instance_init = wpcm450_cpld_init,
};

static void wpcm450_cpld_register_types(void)
{
    type_register_static(&wpcm450_cpld_info);
}

type_init(wpcm450_cpld_register_types)