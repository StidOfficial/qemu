/*
 * ARM Versatile Platform/Application Baseboard System emulation.
 *
 * Copyright (c) 2005-2007 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"

#include "hw/arm/boot.h"
#include "hw/arm/wpcm450.h"
#include "hw/char/serial.h"
#include "hw/loader.h"
#include "hw/misc/unimp.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/units.h"
#include "sysemu/sysemu.h"

#include "hw/char/pl011.h"

/*
 * This covers the whole MMIO space. We'll use this to catch any MMIO accesses
 * that aren't handled by any device.
 */
#define WPCM450_MMIO_BA         (0x80000000)
#define WPCM450_MMIO_SZ         (0x7ffd0000)

/* OTP key storage and fuse strap array */
#define WPCM450_OTP1_BA         (0xf0189000)
#define WPCM450_OTP2_BA         (0xf018a000)

/* Core system modules. */
#define WPCM450_L2C_BA          (0xf03fc000)
#define WPCM450_CPUP_BA         (0xf03fe000)
#define WPCM450_GCR_BA          (0xb0000000)
#define WPCM450_CLK_BA          (0xf0801000)
#define WPCM450_MC_BA           (0xf0824000)
#define WPCM450_RNG_BA          (0xf000b000)

/* USB Host modules */
#define WPCM450_EHCI_BA         (0xf0806000)
#define WPCM450_OHCI_BA         (0xf0807000)

/* ADC Module */
#define WPCM450_ADC_BA          (0xf000c000)

/* Internal AHB SRAM */
#define WPCM450_RAM3_BA         (0xc0008000)
#define WPCM450_RAM3_SZ         (4 * KiB)

/* Memory blocks at the end of the address space */
#define WPCM450_RAM2_BA         (0xfffd0000)
#define WPCM450_RAM2_SZ         (128 * KiB)
#define WPCM450_ROM_BA          (0xffff0000)
#define WPCM450_ROM_SZ          (64 * KiB)


/*
 * Interrupt lines going into the GIC. This does not include internal ARM926EJ-S
 * interrupts.
 */
enum wpcm450_interrupt {
    WPCM450_WDT_IRQ = 1,
    WPCM450_GPIO0_IRQ,
    WPCM450_GPIO1_IRQ,
    WPCM450_GPIO2_IRQ,
    WPCM450_GPIO3_IRQ,
    WPCM450_PECI_IRQ,
    WPCM450_UART0_IRQ,
    WPCM450_UART1_IRQ,
    WPCM450_TIMER0_IRQ = 12,
};

/* Register base address for each Timer Module */
static const hwaddr wpcm450_tim_addr[] = {
    0xb8001000,
};

/* Register base address for each 16550 UART */
static const hwaddr wpcm450_uart_addr[] = {
    0xb8000000,
    0xb8000100,
};

/* Primary interrupt controller.  */

// To Be Remove
static void wpcm450_soc_update(WPCM450State *s)
{
    uint32_t flags;

    flags = s->level & s->mask;
    qemu_set_irq(s->parent[s->irq], flags != 0);
}

static void wpcm450_soc_update_pic(WPCM450State *s)
{
    int i;
    uint32_t mask;

    for (i = 21; i <= 30; i++) {
        mask = 1u << i;
        if (!(s->pic_enable & mask))
            continue;
        qemu_set_irq(s->parent[i], (s->level & mask) != 0);
    }
}

static void wpcm450_soc_set_irq(void *opaque, int irq, int level)
{
    WPCM450State *s = (WPCM450State *)opaque;
    if (level)
        s->level |= 1u << irq;
    else
        s->level &= ~(1u << irq);
    if (s->pic_enable & (1u << irq))
        qemu_set_irq(s->parent[irq], level);
    wpcm450_soc_update(s);
}

static uint64_t wpcm450_soc_read(void *opaque, hwaddr offset,
                             unsigned size)
{
    WPCM450State *s = (WPCM450State *)opaque;

    switch (offset >> 2) {
    case 0: /* STATUS */
        return s->level & s->mask;
    case 1: /* RAWSTAT */
        return s->level;
    case 2: /* ENABLE */
        return s->mask;
    case 4: /* SOFTINT */
        return s->level & 1;
    case 8: /* PICENABLE */
        return s->pic_enable;
    default:
        printf ("wpcm450_soc_read: Bad register offset 0x%x\n", (int)offset);
        return 0;
    }
}

static void wpcm450_soc_write(void *opaque, hwaddr offset,
                          uint64_t value, unsigned size)
{
    WPCM450State *s = (WPCM450State *)opaque;

    switch (offset >> 2) {
    case 2: /* ENSET */
        s->mask |= value;
        break;
    case 3: /* ENCLR */
        s->mask &= ~value;
        break;
    case 4: /* SOFTINTSET */
        if (value)
            s->mask |= 1;
        break;
    case 5: /* SOFTINTCLR */
        if (value)
            s->mask &= ~1u;
        break;
    case 8: /* PICENSET */
        s->pic_enable |= (value & 0x7fe00000);
        wpcm450_soc_update_pic(s);
        break;
    case 9: /* PICENCLR */
        s->pic_enable &= ~value;
        wpcm450_soc_update_pic(s);
        break;
    default:
        printf ("wpcm450_soc_write: Bad register offset 0x%x\n", (int)offset);
        return;
    }
    wpcm450_soc_update(s);
}

static const MemoryRegionOps wpcm450_soc_ops = {
    .read = wpcm450_soc_read,
    .write = wpcm450_soc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/*static qemu_irq wpcm450_irq(WPCM450State *s, int n)
{
    return qdev_get_gpio_in(DEVICE(s->cpu), n);
}*/

void wpcm450_soc_init(MachineState *machine, WPCM450State *s)
{
    int i;

    s->cpu = ARM_CPU(s);

    DeviceState *dev = sysbus_create_varargs("pl190", 0x10140000,
                                qdev_get_gpio_in(DEVICE(s->cpu), ARM_CPU_IRQ),
                                qdev_get_gpio_in(DEVICE(s->cpu), ARM_CPU_FIQ),
                                NULL);

    qemu_irq pic[WPCM450_NUM_IRQ];
    for (i = 0; i < WPCM450_NUM_IRQ; i++) {
        pic[i] = qdev_get_gpio_in(dev, i);
    }

    /* UARTs */
    for (i = 0; i < ARRAY_SIZE(wpcm450_uart_addr); i++) {
        serial_mm_init(get_system_memory(), wpcm450_uart_addr[i], 2,
                       pic[WPCM450_UART0_IRQ + i], 115200,
                       serial_hd(i), DEVICE_LITTLE_ENDIAN);
    }

    /* Timers */
    for (i = 0; i < ARRAY_SIZE(wpcm450_tim_addr); i++) {
        //sysbus_create_simple("sp804", wpcm450_tim_addr[i], pic[WPCM450_TIMER0_IRQ + 1]);
    }
}

// To Be Remove END

static struct arm_boot_info wpcm450_binfo;

void wpcm450_load_kernel(MachineState *machine, WPCM450State *soc)
{
    //WPCM450Class *sc = WPCM450_GET_CLASS(soc);

    wpcm450_binfo.ram_size = machine->ram_size;
    //wpcm450_binfo.nb_cpus = sc->num_cpus;
    wpcm450_binfo.board_id = -1;
    
    arm_load_kernel(soc->cpu, machine, &wpcm450_binfo);
}

static void wpcm450_init(Object *obj)
{
    WPCM450State *s = WPCM450(obj);
    int i;

    object_initialize_child(obj, "tim[*]", &s->tim[0], TYPE_NPCM7XX_TIMER);

    DeviceState *dev = DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    qdev_init_gpio_in(dev, wpcm450_soc_set_irq, WPCM450_NUM_IRQ);
    for (i = 0; i < WPCM450_NUM_IRQ; i++) {
        sysbus_init_irq(sbd, &s->parent[i]);
    }
    s->irq = WPCM450_NUM_IRQ - 1;

    /* Core memory */

    memory_region_init_io(&s->iomem, obj, &wpcm450_soc_ops, s,
                        "vpb-sic", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void wpcm450_realize(DeviceState *dev, Error **errp)
{
    WPCM450State *s = WPCM450(dev);
    //WPCM450Class *nc = WPCM450_GET_CLASS(s);

    if (memory_region_size(s->dram) > WPCM450_DRAM_SZ) {
        error_setg(errp, "%s: NPCM7xx cannot address more than %" PRIu64
                   " MiB of DRAM", __func__, WPCM450_DRAM_SZ / MiB);
        return;
    }

    /* CPU */

    /* Disable security extensions. */
    /*if (object_property_find(OBJECT(s), "has_el3")) {
        object_property_set_bool(OBJECT(s), "has_el3", false, &error_fatal);
    }

    if (!qdev_realize(DEVICE(s->cpu), NULL, errp)) {
        return;
    }*/
}

static void wpcm450_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = wpcm450_realize;
}

static void nuvoton_wpcm450_class_init(ObjectClass *oc, void *data)
{
    WPCM450Class *nc = WPCM450_CLASS(oc);

    /* NPCM730 is optimized for data center use, so no graphics, etc. */
    nc->disabled_modules = 0x00300395;
    nc->num_cpus = 1;
}

static void winbond_wpcm450_class_init(ObjectClass *oc, void *data)
{
    nuvoton_wpcm450_class_init(oc, data);
}

static const TypeInfo wpcm450_soc_types[] = {
    {
        .name           = TYPE_WPCM450,
        .parent         = TYPE_SYS_BUS_DEVICE,
        .instance_size  = sizeof(WPCM450State),
        .instance_init  = wpcm450_init,
        //.class_size     = sizeof(WPCM450Class),
        .class_init     = wpcm450_class_init,
        .abstract       = true,
    }, {
        .name           = TYPE_NUVOTON_WPCM450,
        .parent         = TYPE_WPCM450,
        .class_init     = nuvoton_wpcm450_class_init,
    }, {
        .name           = TYPE_WINBOND_WPCM450,
        .parent         = TYPE_WPCM450,
        .class_init     = winbond_wpcm450_class_init,
    },
};

DEFINE_TYPES(wpcm450_soc_types);