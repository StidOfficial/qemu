/*
 * Nuvoton NPCM7xx SoC family.
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

/* Internal RAMs */
#define WPCM450_RAM0_BA         (0x00000000)
#define WPCM450_RAM0_SZ         (8 * KiB)

#define WPCM450_RAM1_BA         (0xc6000000)
#define WPCM450_RAM1_SZ         (1024)

#define WPCM450_DRAM_MIN_SZ     (128 * MiB)
#define WPCM450_DRAM_MAX_SZ     (WPCM450_DRAM_MIN_SZ * 4)

#define WPCM450_1DRAM_CONFIG    (WPCM450_DRAM_MIN_SZ * 1)
#define WPCM450_2DRAM_CONFIG    (WPCM450_DRAM_MIN_SZ * 2)
#define WPCM450_3DRAM_CONFIG    (WPCM450_DRAM_MIN_SZ * 3)
#define WPCM450_4DRAM_CONFIG    (WPCM450_DRAM_MIN_SZ * 4)

/* The first half of the address space is reserved for DDR3 DRAM. */
#define WPCM450_DRAM0_BA        (0x00000000)
#define WPCM450_DRAM1_BA        (WPCM450_DRAM0_BA + WPCM450_1DRAM_CONFIG)
#define WPCM450_DRAM2_BA        (WPCM450_DRAM0_BA + WPCM450_2DRAM_CONFIG)
#define WPCM450_DRAM3_BA        (WPCM450_DRAM0_BA + WPCM450_3DRAM_CONFIG)
#define WPCM450_DRAM_SZ         WPCM450_DRAM_MIN_SZ

/* Magic addresses for setting up direct kernel booting and SMP boot stubs. */
#define WPCM450_LOADER_START            (0x00000000)  /* Start of SDRAM */
#define WPCM450_SMP_LOADER_START        (0xffff0000)  /* Boot ROM */
#define WPCM450_SMP_BOOTREG_ADDR        (0xf080013c)  /* GCR.SCRPAD */
#define WPCM450_GIC_CPU_IF_ADDR         (0xf03fe100)  /* GIC within A9 */
#define WPCM450_BOARD_SETUP_ADDR        (0xffff1000)  /* Boot ROM */

#define NPCM7XX_NR_PWM_MODULES 2

typedef struct WPCM450Machine {
    MachineState        parent;
    /*
     * PWM fan splitter. each splitter connects to one PWM output and
     * multiple MFT inputs.
     */
    SplitIRQ            fan_splitter[NPCM7XX_NR_PWM_MODULES *
                                     NPCM7XX_PWM_PER_MODULE];
} WPCM450Machine;

#define TYPE_WPCM450_MACHINE MACHINE_TYPE_NAME("wpcm450")
#define WPCM450_MACHINE(obj)                                            \
    OBJECT_CHECK(WPCM450Machine, (obj), TYPE_WPCM450_MACHINE)

typedef struct WPCM450MachineClass {
    MachineClass        parent;

    const char          *soc_type;
} WPCM450MachineClass;

#define WPCM450_MACHINE_CLASS(klass)                                    \
    OBJECT_CLASS_CHECK(WPCM450MachineClass, (klass), TYPE_WPCM450_MACHINE)
#define WPCM450_MACHINE_GET_CLASS(obj)                                  \
    OBJECT_GET_CLASS(WPCM450MachineClass, (obj), TYPE_WPCM450_MACHINE)

typedef struct WPCM450State {
    DeviceState         parent;

    ARMCPU              cpu[WPCM450_MAX_NUM_CPUS];
    A9MPPrivState       a9mpcore;

    MemoryRegion        sram;
    MemoryRegion        irom;
    MemoryRegion        ram3;
    MemoryRegion        *dram;

    NPCM7xxGCRState     gcr;
    NPCM7xxCLKState     clk;
    NPCM7xxTimerCtrlState tim[3];
    NPCM7xxADCState     adc;
    NPCM7xxPWMState     pwm[NPCM7XX_NR_PWM_MODULES];
    NPCM7xxMFTState     mft[8];
    NPCM7xxOTPState     key_storage;
    NPCM7xxOTPState     fuse_array;
    NPCM7xxMCState      mc;
    NPCM7xxRNGState     rng;
    NPCM7xxGPIOState    gpio[8];
    NPCM7xxSMBusState   smbus[16];
    EHCISysBusState     ehci;
    OHCISysBusState     ohci;
    NPCM7xxFIUState     fiu[2];
    NPCM7xxEMCState     emc[2];
} WPCM450State;

#define TYPE_WPCM450    "wpcm450"
#define WPCM450(obj)    OBJECT_CHECK(WPCM450State, (obj), TYPE_WPCM450)

#define TYPE_NPCM730    "npcm730"
#define TYPE_NUVOTON_WPCM450    "nuvoton-wpcm450"

typedef struct WPCM450Class {
    DeviceClass         parent;

    /* Bitmask of modules that are permanently disabled on this chip. */
    uint32_t            disabled_modules;
    /* Number of CPU cores enabled in this SoC class (may be 1 or 2). */
    uint32_t            num_cpus;
} WPCM450Class;

#define WPCM450_CLASS(klass)                                            \
    OBJECT_CLASS_CHECK(WPCM450Class, (klass), TYPE_WPCM450)
#define WPCM450_GET_CLASS(obj)                                          \
    OBJECT_GET_CLASS(WPCM450Class, (obj), TYPE_WPCM450)

/**
 * wpcm450_load_kernel - Loads memory with everything needed to boot
 * @machine - The machine containing the SoC to be booted.
 * @soc - The SoC containing the CPU to be booted.
 *
 * This will set up the ARM boot info structure for the specific NPCM7xx
 * derivative and call arm_load_kernel() to set up loading of the kernel, etc.
 * into memory, if requested by the user.
 */
void wpcm450_load_kernel(MachineState *machine, WPCM450State *soc);

#endif /* WPCM450_H */
