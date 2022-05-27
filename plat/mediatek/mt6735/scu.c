/*
 * Copyright (c) 2015, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch.h>
#include <lib/mmio.h>

#include <mcucfg.h>

void disable_scu(unsigned long mpidr)
{
		mmio_setbits_32((uintptr_t)&mt6735_mcucfg->mp0_axi_config,
			MP0_ACINACTM);
}

void enable_scu(unsigned long mpidr)
{
		mmio_clrbits_32((uintptr_t)&mt6735_mcucfg->mp0_axi_config,
			MP0_ACINACTM);
}
