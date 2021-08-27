/*
 * Machine definitions for boards featuring an WPCM450 SoC.
 *
 * Copyright 2020 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 */

#include "qemu/osdep.h"

#include "hw/arm/wpcm450.h"
#include "hw/core/cpu.h"
#include "hw/i2c/i2c_mux_pca954x.h"
#include "hw/i2c/smbus_eeprom.h"
#include "hw/loader.h"
#include "hw/qdev-core.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "qemu/datadir.h"
#include "qemu/units.h"

#define NPCM750_EVB_POWER_ON_STRAPS 0x00001ff7
#define QUANTA_GSJ_POWER_ON_STRAPS 0x00001fff
#define QUANTA_GBS_POWER_ON_STRAPS 0x000017ff

#ifdef IGNORE_IROM
static const char npcm7xx_default_bootrom[] = "npcm7xx_bootrom.bin";

static void npcm7xx_load_bootrom(MachineState *machine, WPCM450State *soc)
{
    const char *bios_name = machine->firmware ?: npcm7xx_default_bootrom;
    g_autofree char *filename = NULL;
    int ret;

    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    if (!filename) {
        error_report("Could not find ROM image '%s'", bios_name);
        if (!machine->kernel_filename) {
            /* We can't boot without a bootrom or a kernel image. */
            exit(1);
        }
        return;
    }
    ret = load_image_mr(filename, &soc->irom);
    if (ret < 0) {
        error_report("Failed to load ROM image '%s'", filename);
        exit(1);
    }
}
#endif

#ifdef IGNORE_FIU
static void npcm7xx_connect_flash(NPCM7xxFIUState *fiu, int cs_no,
                                  const char *flash_type, DriveInfo *dinfo)
{
    DeviceState *flash;
    qemu_irq flash_cs;

    flash = qdev_new(flash_type);
    if (dinfo) {
        qdev_prop_set_drive(flash, "drive", blk_by_legacy_dinfo(dinfo));
    }
    qdev_realize_and_unref(flash, BUS(fiu->spi), &error_fatal);

    flash_cs = qdev_get_gpio_in_named(flash, SSI_GPIO_CS, 0);
    qdev_connect_gpio_out_named(DEVICE(fiu), "cs", cs_no, flash_cs);
}
#endif

static void npcm7xx_connect_dram(WPCM450State *soc, MemoryRegion *dram)
{
    MemoryRegion *smem = get_system_memory();
    MemoryRegion *dram1 = g_new(MemoryRegion, 1);
    MemoryRegion *dram2 = g_new(MemoryRegion, 1);
    MemoryRegion *dram3 = g_new(MemoryRegion, 1);

    memory_region_init_ram(dram1, NULL, "dram1", WPCM450_DRAM_MAX_SZ, &error_fatal);
    memory_region_init_ram(dram2, NULL, "dram2", WPCM450_DRAM_MAX_SZ, &error_fatal);
    memory_region_init_ram(dram3, NULL, "dram3", WPCM450_DRAM_MAX_SZ, &error_fatal);

    memory_region_add_subregion_overlap(smem, WPCM450_DRAM0_BA, dram, 1);

    if(dram->size == WPCM450_1DRAM_CONFIG)
    {
        memory_region_init_alias(dram1, NULL, "dram1", dram, 0x0, WPCM450_DRAM_MIN_SZ);
        memory_region_add_subregion(smem, WPCM450_DRAM1_BA, dram1);

        memory_region_init_alias(dram2, NULL, "dram2", dram, 0x0, WPCM450_DRAM_MIN_SZ);
        memory_region_add_subregion(smem, WPCM450_DRAM2_BA, dram2);

        memory_region_init_alias(dram3, NULL, "dram3", dram, 0x0, WPCM450_DRAM_MIN_SZ);
        memory_region_add_subregion(smem, WPCM450_DRAM3_BA, dram3);
    }

    object_property_set_link(OBJECT(soc), "dram-mr", OBJECT(dram),
                             &error_abort);
}

static WPCM450State *wpcm450_create_soc(MachineState *machine,
                                        uint32_t hw_straps)
{
    WPCM450MachineClass *nmc = WPCM450_MACHINE_GET_CLASS(machine);
    MachineClass *mc = MACHINE_CLASS(nmc);
    Object *obj;
    MemoryRegion *ram0 = g_new(MemoryRegion, 1);
    MemoryRegion *ram1 = g_new(MemoryRegion, 1);

    if (strcmp(machine->cpu_type, mc->default_cpu_type) != 0) {
        error_report("This board can only be used with %s",
                     mc->default_cpu_type);
        exit(1);
    }

    obj = object_new_with_props(nmc->soc_type, OBJECT(machine), "soc",
                                &error_abort, NULL);
#ifdef IGNORE_GCR
    object_property_set_uint(obj, "power-on-straps", hw_straps, &error_abort);
#endif

    /* Internal RAM0 */
    memory_region_init_ram(ram0, NULL, "ram0", WPCM450_RAM0_SZ, &error_fatal);
    memory_region_add_subregion(get_system_memory(), WPCM450_RAM0_BA, ram0);

    /* Internal RAM1 */
    memory_region_init_ram(ram1, NULL, "ram1", WPCM450_RAM1_SZ, &error_fatal);
    memory_region_add_subregion(get_system_memory(), WPCM450_RAM1_BA, ram1);

    return WPCM450(obj);
}

#ifdef IGNORE_SMBUS
static I2CBus *npcm7xx_i2c_get_bus(WPCM450State *soc, uint32_t num)
{
    g_assert(num < ARRAY_SIZE(soc->smbus));
    return I2C_BUS(qdev_get_child_bus(DEVICE(&soc->smbus[num]), "i2c-bus"));
}
#endif

#ifdef IGNORE
static void at24c_eeprom_init(WPCM450State *soc, int bus, uint8_t addr,
                              uint32_t rsize)
{
    I2CBus *i2c_bus = npcm7xx_i2c_get_bus(soc, bus);
    I2CSlave *i2c_dev = i2c_slave_new("at24c-eeprom", addr);
    DeviceState *dev = DEVICE(i2c_dev);

    qdev_prop_set_uint32(dev, "rom-size", rsize);
    i2c_slave_realize_and_unref(i2c_dev, i2c_bus, &error_abort);
}
#endif

#ifdef IGNORE_PWM
static void npcm7xx_init_pwm_splitter(WPCM450Machine *machine,
                                      WPCM450State *soc, const int *fan_counts)
{
    SplitIRQ *splitters = machine->fan_splitter;

    /*
     * PWM 0~3 belong to module 0 output 0~3.
     * PWM 4~7 belong to module 1 output 0~3.
     */
    for (int i = 0; i < NPCM7XX_NR_PWM_MODULES; ++i) {
        for (int j = 0; j < NPCM7XX_PWM_PER_MODULE; ++j) {
            int splitter_no = i * NPCM7XX_PWM_PER_MODULE + j;
            DeviceState *splitter;

            if (fan_counts[splitter_no] < 1) {
                continue;
            }
            object_initialize_child(OBJECT(machine), "fan-splitter[*]",
                                    &splitters[splitter_no], TYPE_SPLIT_IRQ);
            splitter = DEVICE(&splitters[splitter_no]);
            qdev_prop_set_uint16(splitter, "num-lines",
                                 fan_counts[splitter_no]);
            qdev_realize(splitter, NULL, &error_abort);
            qdev_connect_gpio_out_named(DEVICE(&soc->pwm[i]), "duty-gpio-out",
                                        j, qdev_get_gpio_in(splitter, 0));
        }
    }
}

static void npcm7xx_connect_pwm_fan(WPCM450State *soc, SplitIRQ *splitter,
                                    int fan_no, int output_no)
{
    DeviceState *fan;
    int fan_input;
    qemu_irq fan_duty_gpio;

    g_assert(fan_no >= 0 && fan_no <= NPCM7XX_MFT_MAX_FAN_INPUT);
    /*
     * Fan 0~1 belong to module 0 input 0~1.
     * Fan 2~3 belong to module 1 input 0~1.
     * ...
     * Fan 14~15 belong to module 7 input 0~1.
     * Fan 16~17 belong to module 0 input 2~3.
     * Fan 18~19 belong to module 1 input 2~3.
     */
    if (fan_no < 16) {
        fan = DEVICE(&soc->mft[fan_no / 2]);
        fan_input = fan_no % 2;
    } else {
        fan = DEVICE(&soc->mft[(fan_no - 16) / 2]);
        fan_input = fan_no % 2 + 2;
    }

    /* Connect the Fan to PWM module */
    fan_duty_gpio = qdev_get_gpio_in_named(fan, "duty", fan_input);
    qdev_connect_gpio_out(DEVICE(splitter), output_no, fan_duty_gpio);
}
#endif

#ifdef IGNORE_SMBUS
static void idrac6_bmc_i2c_init(WPCM450State *soc)
{
    /* lm75 temperature sensor on SVB, tmp105 is compatible */
    i2c_slave_create_simple(npcm7xx_i2c_get_bus(soc, 0), "tmp105", 0x48);
    /* lm75 temperature sensor on EB, tmp105 is compatible */
    i2c_slave_create_simple(npcm7xx_i2c_get_bus(soc, 1), "tmp105", 0x48);
    /* tmp100 temperature sensor on EB, tmp105 is compatible */
    i2c_slave_create_simple(npcm7xx_i2c_get_bus(soc, 2), "tmp105", 0x48);
    /* tmp100 temperature sensor on SVB, tmp105 is compatible */
    i2c_slave_create_simple(npcm7xx_i2c_get_bus(soc, 6), "tmp105", 0x48);
}
#endif

#ifdef IGNORE_PWM
static void idrac6_bmc_fan_init(WPCM450Machine *machine, WPCM450State *soc)
{
    SplitIRQ *splitter = machine->fan_splitter;
    static const int fan_counts[] = {2, 2, 2, 2, 2, 2, 2, 2};

    npcm7xx_init_pwm_splitter(machine, soc, fan_counts);
    npcm7xx_connect_pwm_fan(soc, &splitter[0], 0x00, 0);
    npcm7xx_connect_pwm_fan(soc, &splitter[0], 0x01, 1);
    npcm7xx_connect_pwm_fan(soc, &splitter[1], 0x02, 0);
    npcm7xx_connect_pwm_fan(soc, &splitter[1], 0x03, 1);
    npcm7xx_connect_pwm_fan(soc, &splitter[2], 0x04, 0);
    npcm7xx_connect_pwm_fan(soc, &splitter[2], 0x05, 1);
    npcm7xx_connect_pwm_fan(soc, &splitter[3], 0x06, 0);
    npcm7xx_connect_pwm_fan(soc, &splitter[3], 0x07, 1);
    npcm7xx_connect_pwm_fan(soc, &splitter[4], 0x08, 0);
    npcm7xx_connect_pwm_fan(soc, &splitter[4], 0x09, 1);
    npcm7xx_connect_pwm_fan(soc, &splitter[5], 0x0a, 0);
    npcm7xx_connect_pwm_fan(soc, &splitter[5], 0x0b, 1);
    npcm7xx_connect_pwm_fan(soc, &splitter[6], 0x0c, 0);
    npcm7xx_connect_pwm_fan(soc, &splitter[6], 0x0d, 1);
    npcm7xx_connect_pwm_fan(soc, &splitter[7], 0x0e, 0);
    npcm7xx_connect_pwm_fan(soc, &splitter[7], 0x0f, 1);
}
#endif

#ifdef IGNORE
static void quanta_gsj_i2c_init(WPCM450State *soc)
{
    /* GSJ machine have 4 max31725 temperature sensors, tmp105 is compatible. */
    /*i2c_slave_create_simple(npcm7xx_i2c_get_bus(soc, 1), "tmp105", 0x5c);
    i2c_slave_create_simple(npcm7xx_i2c_get_bus(soc, 2), "tmp105", 0x5c);
    i2c_slave_create_simple(npcm7xx_i2c_get_bus(soc, 3), "tmp105", 0x5c);
    i2c_slave_create_simple(npcm7xx_i2c_get_bus(soc, 4), "tmp105", 0x5c);

    at24c_eeprom_init(soc, 9, 0x55, 8192);
    at24c_eeprom_init(soc, 10, 0x55, 8192);*/

    /*
     * i2c-11:
     * - power-brick@36: delta,dps800
     * - hotswap@15: ti,lm5066i
     */

    /*
     * i2c-12:
     * - ucd90160@6b
     */

    i2c_slave_create_simple(npcm7xx_i2c_get_bus(soc, 15), "pca9548", 0x75);
}
#endif

#ifdef IGNORE_PWM
static void quanta_gsj_fan_init(WPCM450Machine *machine, WPCM450State *soc)
{
    SplitIRQ *splitter = machine->fan_splitter;
    static const int fan_counts[] = {2, 2, 2, 0, 0, 0, 0, 0};

    npcm7xx_init_pwm_splitter(machine, soc, fan_counts);
    npcm7xx_connect_pwm_fan(soc, &splitter[0], 0x00, 0);
    npcm7xx_connect_pwm_fan(soc, &splitter[0], 0x01, 1);
    npcm7xx_connect_pwm_fan(soc, &splitter[1], 0x02, 0);
    npcm7xx_connect_pwm_fan(soc, &splitter[1], 0x03, 1);
    npcm7xx_connect_pwm_fan(soc, &splitter[2], 0x04, 0);
    npcm7xx_connect_pwm_fan(soc, &splitter[2], 0x05, 1);
}
#endif

#ifdef IGNORE
static void quanta_gbs_i2c_init(WPCM450State *soc)
{
    /*
     * i2c-0:
     *     pca9546@71
     *
     * i2c-1:
     *     pca9535@24
     *     pca9535@20
     *     pca9535@21
     *     pca9535@22
     *     pca9535@23
     *     pca9535@25
     *     pca9535@26
     *
     * i2c-2:
     *     sbtsi@4c
     *
     * i2c-5:
     *     atmel,24c64@50 mb_fru
     *     pca9546@71
     *         - channel 0: max31725@54
     *         - channel 1: max31725@55
     *         - channel 2: max31725@5d
     *                      atmel,24c64@51 fan_fru
     *         - channel 3: atmel,24c64@52 hsbp_fru
     *
     * i2c-6:
     *     pca9545@73
     *
     * i2c-7:
     *     pca9545@72
     *
     * i2c-8:
     *     adi,adm1272@10
     *
     * i2c-9:
     *     pca9546@71
     *         - channel 0: isil,isl68137@60
     *         - channel 1: isil,isl68137@61
     *         - channel 2: isil,isl68137@63
     *         - channel 3: isil,isl68137@45
     *
     * i2c-10:
     *     pca9545@71
     *
     * i2c-11:
     *     pca9545@76
     *
     * i2c-12:
     *     maxim,max34451@4e
     *     isil,isl68137@5d
     *     isil,isl68137@5e
     *
     * i2c-14:
     *     pca9545@70
     */
}
#endif

static void idrac6_bmc_init(MachineState *machine)
{
    WPCM450State *soc;

    soc = wpcm450_create_soc(machine, NPCM750_EVB_POWER_ON_STRAPS);
    npcm7xx_connect_dram(soc, machine->ram);
    qdev_realize(DEVICE(soc), NULL, &error_fatal);

#ifdef IGNORE_IROM
    npcm7xx_load_bootrom(machine, soc);
#endif
#ifdef IGNORE_FIU
    npcm7xx_connect_flash(&soc->fiu[0], 0, "w25q256", drive_get(IF_MTD, 0, 0));
#endif
#ifdef IGNORE_SMBUS
    idrac6_bmc_i2c_init(soc);
#endif
#ifdef IGNORE_PWM
    idrac6_bmc_fan_init(WPCM450_MACHINE(machine), soc);
#endif
    wpcm450_load_kernel(machine, soc);
}

#ifdef IGNORE
static void quanta_gsj_init(MachineState *machine)
{
    WPCM450State *soc;

    soc = npcm7xx_create_soc(machine, QUANTA_GSJ_POWER_ON_STRAPS);
    npcm7xx_connect_dram(soc, machine->ram);
    qdev_realize(DEVICE(soc), NULL, &error_fatal);

    npcm7xx_load_bootrom(machine, soc);
    npcm7xx_connect_flash(&soc->fiu[0], 0, "mx25l25635e",
                          drive_get(IF_MTD, 0, 0));
    quanta_gsj_i2c_init(soc);
    quanta_gsj_fan_init(WPCM450_MACHINE(machine), soc);
    wpcm450_load_kernel(machine, soc);
}

static void quanta_gbs_init(MachineState *machine)
{
    WPCM450State *soc;

    soc = npcm7xx_create_soc(machine, QUANTA_GBS_POWER_ON_STRAPS);
    npcm7xx_connect_dram(soc, machine->ram);
    qdev_realize(DEVICE(soc), NULL, &error_fatal);

    npcm7xx_load_bootrom(machine, soc);

    npcm7xx_connect_flash(&soc->fiu[0], 0, "mx66u51235f",
                          drive_get(IF_MTD, 0, 0));

    quanta_gbs_i2c_init(soc);
    wpcm450_load_kernel(machine, soc);
}
#endif

static void wpcm450_set_soc_type(WPCM450MachineClass *nmc, const char *type)
{
    WPCM450Class *sc = WPCM450_CLASS(object_class_by_name(type));
    MachineClass *mc = MACHINE_CLASS(nmc);

    nmc->soc_type = type;
    mc->default_cpus = mc->min_cpus = mc->max_cpus = sc->num_cpus;
}

static void wpcm450_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->no_parallel = 1;
    mc->default_ram_id = "dram0";
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm926");
}

/*
 * Schematics:
 * https://github.com/Nuvoton-Israel/nuvoton-info/blob/master/npcm7xx-poleg/evaluation-board/board_deliverables/NPCM750x_EB_ver.A1.1_COMPLETE.pdf
 */
static void idrac6_bmc_machine_class_init(ObjectClass *oc, void *data)
{
    WPCM450MachineClass *nmc = WPCM450_MACHINE_CLASS(oc);
    MachineClass *mc = MACHINE_CLASS(oc);

    wpcm450_set_soc_type(nmc, TYPE_NUVOTON_WPCM450);

    mc->desc = "Nuvoton WPCM450 iDRAC6 BMC (ARM926EJ-S)";
    mc->init = idrac6_bmc_init;
    mc->default_ram_size = WPCM450_1DRAM_CONFIG;
};

#ifdef IGNORE
static void gsj_machine_class_init(ObjectClass *oc, void *data)
{
    WPCM450MachineClass *nmc = WPCM450_MACHINE_CLASS(oc);
    MachineClass *mc = MACHINE_CLASS(oc);

    wpcm450_set_soc_type(nmc, TYPE_NPCM730);

    mc->desc = "Quanta GSJ (Cortex-A9)";
    mc->init = quanta_gsj_init;
    mc->default_ram_size = 512 * MiB;
};

static void gbs_bmc_machine_class_init(ObjectClass *oc, void *data)
{
    WPCM450MachineClass *nmc = WPCM450_MACHINE_CLASS(oc);
    MachineClass *mc = MACHINE_CLASS(oc);

    wpcm450_set_soc_type(nmc, TYPE_NPCM730);

    mc->desc = "Quanta GBS (Cortex-A9)";
    mc->init = quanta_gbs_init;
    mc->default_ram_size = 1 * GiB;
}
#endif

static const TypeInfo wpcm450_machine_types[] = {
    {
        .name           = TYPE_WPCM450_MACHINE,
        .parent         = TYPE_MACHINE,
        .instance_size  = sizeof(WPCM450Machine),
        .class_size     = sizeof(WPCM450MachineClass),
        .class_init     = wpcm450_machine_class_init,
        .abstract       = true,
    }, {
        .name           = MACHINE_TYPE_NAME("idrac6-bmc"),
        .parent         = TYPE_WPCM450_MACHINE,
        .class_init     = idrac6_bmc_machine_class_init,
    },/* {
        .name           = MACHINE_TYPE_NAME("quanta-gsj"),
        .parent         = TYPE_WPCM450_MACHINE,
        .class_init     = gsj_machine_class_init,
    }, {
        .name           = MACHINE_TYPE_NAME("quanta-gbs-bmc"),
        .parent         = TYPE_WPCM450_MACHINE,
        .class_init     = gbs_bmc_machine_class_init,
    },*/
};

DEFINE_TYPES(wpcm450_machine_types)
