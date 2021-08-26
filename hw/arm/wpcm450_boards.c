#include "qemu/osdep.h"

#include "hw/arm/wpcm450.h"
#include "hw/core/cpu.h"
#include "hw/i2c/smbus_eeprom.h"
#include "hw/loader.h"
#include "hw/qdev-core.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "qemu/datadir.h"
#include "qemu/units.h"

#define TYPE_IDRAC6_BMC_MACHINE MACHINE_TYPE_NAME("idrac6-bmc")

static void wpcm450_connect_dram(WPCM450State *soc, MemoryRegion *dram)
{
    memory_region_add_subregion(get_system_memory(), WPCM450_DRAM_BA, dram);

    /*object_property_set_link(OBJECT(soc), "dram-mr", OBJECT(dram),
                                &error_abort);*/
}

static WPCM450State *wpcm450_create_soc(MachineState *machine)
{
    WPCM450MachineClass *nmc = WPCM450_MACHINE_GET_CLASS(machine);
    MachineClass *mc = MACHINE_CLASS(nmc);
    Object *obj;

    if (strcmp(machine->cpu_type, mc->default_cpu_type) != 0) {
        error_report("This board can only be used with %s",
                     mc->default_cpu_type);
        exit(1);
    }

    /*obj = object_new_with_props(nmc->soc_type, OBJECT(machine), "soc",
                                &error_abort, NULL);
    return WPCM450(obj);*/
    obj = object_new_with_props(machine->cpu_type, OBJECT(machine), "soc",
                                &error_abort, NULL);
    return OBJECT_CHECK(WPCM450State, (obj), machine->cpu_type);
}

static void idrac6_bmc_init(MachineState *machine)
{
    WPCM450State *soc;

    soc = wpcm450_create_soc(machine);
    wpcm450_connect_dram(soc, machine->ram);
    qdev_realize(DEVICE(soc), NULL, &error_fatal);

    if(1)
        wpcm450_soc_init(machine, soc);

    wpcm450_load_kernel(machine, soc);
}

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
    mc->default_ram_id = "wpcm450.ram";
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm926");
}

static void idrac6_bmc_machine_class_init(ObjectClass *oc, void *data)
{
    WPCM450MachineClass *nmc = WPCM450_MACHINE_CLASS(oc);
    MachineClass *mc = MACHINE_CLASS(oc);

    wpcm450_set_soc_type(nmc, TYPE_NUVOTON_WPCM450);

    mc->desc = "Dell iDRAC6 BMC (ARM926EJ-S)";
    mc->init = idrac6_bmc_init;
    mc->block_default_type = IF_SCSI;
    mc->ignore_memory_transaction_failures = true;
    mc->default_ram_size = 256 * MiB;
}

static const TypeInfo wpcm450_machine_types[] = {
    {
        .name           = TYPE_WPCM450_MACHINE,
        .parent         = TYPE_MACHINE,
        .instance_size  = sizeof(WPCM450Machine),
        .class_size     = sizeof(WPCM450MachineClass),
        .class_init     = wpcm450_machine_class_init,
        .abstract       = true,
    },
    {
        .name           = TYPE_IDRAC6_BMC_MACHINE,
        .parent         = TYPE_WPCM450_MACHINE,
        .class_init     = idrac6_bmc_machine_class_init,
    },
};

DEFINE_TYPES(wpcm450_machine_types)