#ifndef WPCM450_H
#define WPCM450_H

#include "hw/boards.h"
#include "hw/adc/npcm7xx_adc.h"
#include "hw/core/split-irq.h"
#include "hw/cpu/a9mpcore.h"
#include "hw/gpio/npcm7xx_gpio.h"
#include "hw/i2c/npcm7xx_smbus.h"
#include "hw/mem/npcm7xx_mc.h"
#include "hw/misc/npcm7xx_clk.h"
#include "hw/misc/npcm7xx_gcr.h"
#include "hw/misc/npcm7xx_mft.h"
#include "hw/misc/npcm7xx_pwm.h"
#include "hw/misc/npcm7xx_rng.h"
#include "hw/net/npcm7xx_emc.h"
#include "hw/nvram/npcm7xx_otp.h"
#include "hw/timer/npcm7xx_timer.h"
#include "hw/ssi/npcm7xx_fiu.h"
#include "hw/usb/hcd-ehci.h"
#include "hw/usb/hcd-ohci.h"
#include "target/arm/cpu.h"

#define WPCM450_MAX_NUM_CPUS    (1)

/* The first half of the address space is reserved for DDR3 DRAM. */
#define WPCM450_DRAM_BA         (0x00000000)
#define WPCM450_DRAM_SZ         (256 * MiB)

/* Total number of GIC interrupts, including internal ARM926EJ-S interrupts. */
#define WPCM450_NUM_IRQ         (32)

typedef struct WPCM450Machine {
    MachineState        parent;
} WPCM450Machine;

#define TYPE_WPCM450_MACHINE MACHINE_TYPE_NAME(TYPE_WPCM450)
#define WPCM450_MACHINE(obj) \
    OBJECT_CHECK(NPCM7xxMachine, (obj), TYPE_WPCM450_MACHINE)                                          \

typedef struct WPCM450MachineClass {
    MachineClass        parent;

    const char          *soc_type;
} WPCM450MachineClass;

#define WPCM450_MACHINE_CLASS(klass) \
    OBJECT_CLASS_CHECK(WPCM450MachineClass, (klass), TYPE_WPCM450_MACHINE)
#define WPCM450_MACHINE_GET_CLASS(obj) \
    OBJECT_GET_CLASS(WPCM450MachineClass, (obj), TYPE_WPCM450_MACHINE)

struct WPCM450State {
    SysBusDevice parent_obj;

    ARMCPU *cpu;
    A9MPPrivState a9mpcore;

    MemoryRegion *dram;

    NPCM7xxTimerCtrlState tim[3];
    NPCM7xxCLKState clk;

    MemoryRegion iomem;
    uint32_t level;
    uint32_t mask;
    uint32_t pic_enable;
    qemu_irq parent[WPCM450_NUM_IRQ];
    int irq;
};

#define TYPE_WPCM450 "wpcm450"
#define TYPE_NUVOTON_WPCM450 "nuvoton-wpcm450"
#define TYPE_WINBOND_WPCM450 "winbond-wpcm450"

OBJECT_DECLARE_SIMPLE_TYPE(WPCM450State, WPCM450)

typedef struct WPCM450Class {
    DeviceClass         parent;

    /* Bitmask of modules that are permanently disabled on this chip. */
    uint32_t            disabled_modules;
    /* Number of CPU cores enabled in this SoC class (may be 1 or 2). */
    uint32_t            num_cpus;
} WPCM450Class;

#define WPCM450_CLASS(klass) \
    OBJECT_CLASS_CHECK(WPCM450Class, (klass), TYPE_WPCM450)
#define WPCM450_GET_CLASS(obj) \
    OBJECT_GET_CLASS(WPCM450Class, (obj), TYPE_WPCM450)

/**
 * wpcm450_load_kernel - Loads memory with everything needed to boot
 * @machine - The machine containing the SoC to be booted.
 * @soc - The SoC containing the CPU to be booted.
 *
 * This will set up the ARM boot info structure for the specific WPCM450
 * derivative and call arm_load_kernel() to set up loading of the kernel, etc.
 * into memory, if requested by the user.
 */
void wpcm450_load_kernel(MachineState *machine, WPCM450State *soc);

// Temp
void wpcm450_soc_init(MachineState *machine, WPCM450State *s);

#endif