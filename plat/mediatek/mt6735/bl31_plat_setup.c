/*
 * Copyright (c) 2013-2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>

#include <common/bl_common.h>
#include <common/debug.h>
#include <common/desc_image_load.h>
#include <drivers/generic_delay_timer.h>
#include <drivers/ti/uart/uart_16550.h>
#include <lib/mmio.h>
#include <plat/arm/common/plat_arm.h>
#include <plat/common/common_def.h>
#include <plat/common/platform.h>

#include <mcucfg.h>
#include <mtcmos.h>
#include <mtk_plat_common.h>
#include <plat_private.h>
#include <spm.h>

static entry_point_info_t bl32_ep_info;
static entry_point_info_t bl33_ep_info;


static void platform_setup_cpu(void)
{
	/* set LITTLE cores arm64 boot mode */
	mmio_setbits_32((uintptr_t)&mt6735_mcucfg->mp0_misc_config[3],
			MP0_CPUCFG_64BIT);
}

static void platform_setup_sram(void)
{
	/* protect BL31 memory from non-secure read/write access */
	mmio_write_32(SRAMROM_SEC_ADDR, (uint32_t)(BL31_END + 0x3ff) & 0x3fc00);
	mmio_write_32(SRAMROM_SEC_CTRL, 0x10000ff9);
}

/*******************************************************************************
 * Return a pointer to the 'entry_point_info' structure of the next image for
 * the security state specified. BL33 corresponds to the non-secure image type
 * while BL32 corresponds to the secure image type. A NULL pointer is returned
 * if the image does not exist.
 ******************************************************************************/
entry_point_info_t *bl31_plat_get_next_image_ep_info(uint32_t type)
{
	entry_point_info_t *next_image_info;

	next_image_info = (type == NON_SECURE) ? &bl33_ep_info : &bl32_ep_info;
	assert(next_image_info->h.type == PARAM_EP);

	/* None of the images on this platform can have 0x0 as the entrypoint */
	if (next_image_info->pc)
		return next_image_info;
	else
		return NULL;
}

/*******************************************************************************
 * Perform any BL3-1 early platform setup. Here is an opportunity to copy
 * parameters passed by the calling EL (S-EL1 in BL2 & EL3 in BL1) before they
 * are lost (potentially). This needs to be done before the MMU is initialized
 * so that the memory layout can be used while creating page tables.
 * BL2 has flushed this information to memory, so we are guaranteed to pick up
 * good data.
 ******************************************************************************/
void bl31_early_platform_setup2(u_register_t arg0, u_register_t arg1,
				u_register_t arg2, u_register_t arg3)
{
	static console_t console;
    
    config_l2_size();
	console_16550_register(MT6735_UART0_BASE, MT6735_UART_CLOCK, MT6735_BAUDRATE, &console);

	VERBOSE("bl31_setup\n");

	bl31_params_parse_helper(arg0, &bl32_ep_info, &bl33_ep_info);
}

/*******************************************************************************
 * Perform any BL3-1 platform setup code
 ******************************************************************************/
void bl31_platform_setup(void)
{
    start_devapc();
    emimpu_setup();
	platform_setup_cpu();
	platform_setup_sram();

	generic_delay_timer_init();

	/* Initialize the gic cpu and distributor interfaces */
	plat_arm_gic_driver_init();
	plat_arm_gic_init();

	/* Initialize spm at boot time */
	//spm_boot_init();
}

/*******************************************************************************
 * Perform the very early platform specific architectural setup here. At the
 * moment this is only intializes the mmu in a quick and dirty way.
 ******************************************************************************/
void bl31_plat_arch_setup(void)
{
	plat_configure_mmu_el3(BL_CODE_BASE,
			       BL_COHERENT_RAM_END - BL_CODE_BASE,
			       BL_CODE_BASE,
			       BL_CODE_END,
			       BL_COHERENT_RAM_BASE,
			       BL_COHERENT_RAM_END);
}

