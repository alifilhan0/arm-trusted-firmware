/*
 * Copyright (c) 2015-2017 MICROTRUST Incorporated
 * All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 * Neither the name of MICROTRUST nor the names of its contributors may
 * be used to endorse or promote products derived from this software without
 * specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include <assert.h>
#include <debug.h>
#include <string.h>

#include <platform.h>
#include "plat_def.h"
#include "plat_private.h"
#include <arch_helpers.h>
#include <plat_config.h>
#include <fiq_smp_call.h>
#include <runtime_svc.h>
#include <bl_common.h>
#include <teei_private.h>
#include <gic_v2.h>
#include <gic_v3.h>
#include <stdio.h>
#include "plat_teei.h"
#include <rng.h>

static tee_arg_t_ptr teeiBootCfg;
static tee_keys_t_ptr teeiKeys;


unsigned int sec_exc[8];
unsigned int nsec_exc[8];

unsigned int SEC_EXC_CNT;
unsigned int NSEC_EXC_CNT;

unsigned int NSEC_TIMER;
unsigned int NSEC_UART;

unsigned int SEC_TIMER;
unsigned int SEC_SPI0 = 150;
unsigned int SEC_APP_INTR;
unsigned int SEC_DRV_INTR;
unsigned int SEC_RDRV_INTR;
unsigned int SEC_TDUMP_INTR;

int s_wdt_lock;

void disable_group(unsigned int grp)
{
	unsigned int gicc_base, pre;

	gicc_base = get_plat_config()->gicc_base;

	pre = gicc_read_ctlr(gicc_base);
	pre = pre & (~(1 << grp));
	gicc_write_ctlr(gicc_base, pre);
}

void enable_group(unsigned int grp)
{
	unsigned int gicc_base, pre;

	gicc_base = get_plat_config()->gicc_base;

	pre = gicc_read_ctlr(gicc_base);
	pre = pre | (1 << grp);
	gicc_write_ctlr(gicc_base, pre);
}

void disable_ns_exc_intr(unsigned int gicd_base)
{
	unsigned int index;

	for (index = 0; index < NSEC_EXC_CNT; index++)
		gicd_set_icenabler(gicd_base, nsec_exc[index]);
}

void enable_ns_exc_intr(unsigned int gicd_base)
{
	unsigned int index;

	for (index = 0; index < NSEC_EXC_CNT; index++)
		gicd_set_isenabler(gicd_base, nsec_exc[index]);
}

void disable_s_exc_intr(unsigned int gicd_base)
{
	unsigned int index, val;
	for (index = 0; index < SEC_EXC_CNT; index++) {
		gicd_set_icenabler(gicd_base, sec_exc[index]);
		val = gicd_read_itargetsr(gicd_base, sec_exc[index]);
		gicd_write_itargetsr(gicd_base, sec_exc[index],
					val & ~(0xff << ((sec_exc[index] % 4) * 8)));
	}
}

void enable_s_exc_intr(unsigned int gicd_base)
{
	unsigned int index;

	for (index = 0; index < SEC_EXC_CNT; index++)
		gicd_set_isenabler(gicd_base, sec_exc[index]);
}

void prepare_gic_for_nsec_boot(void)
{
	unsigned int gicd_base;

	gicd_base = get_plat_config()->gicd_base;
	disable_s_exc_intr(gicd_base);
}

unsigned int get_irq_target(unsigned int irq)
{
	unsigned int gicd_base = get_plat_config()->gicd_base;

	return gicd_read_itargetsr(gicd_base, irq);
}

void prepare_gic_for_sec_boot(void)
{
	unsigned int gicd_base, index;

	gicd_base = get_plat_config()->gicd_base;
	for (index = 0; index < SEC_EXC_CNT; index++)
		gicd_set_icenabler(gicd_base, sec_exc[index]);
}

void migrate_gic_context(uint32_t secure_state)
{
	unsigned int gicd_base, index, val;

	gicd_base = get_plat_config()->gicd_base;

	if (secure_state == SECURE) {
		disable_ns_exc_intr(gicd_base);
		enable_s_exc_intr(gicd_base);
		if (TEEI_STATE < TEEI_BUF_READY)
			disable_group(1);
		for (index = 0; index < SEC_EXC_CNT; index++) {
			val = gicd_read_itargetsr(gicd_base, sec_exc[index]);
			gicd_write_itargetsr(gicd_base,
						sec_exc[index],
						val & ~(0xff << ((sec_exc[index] % 4) * 8)));
			gicd_set_itargetsr(gicd_base,
						sec_exc[index],
						platform_get_core_pos(read_mpidr()));
			gicd_set_isenabler(gicd_base, sec_exc[index]);
		}
	} else {
		enable_ns_exc_intr(gicd_base);
		disable_s_exc_intr(gicd_base);
		if (TEEI_STATE < TEEI_BUF_READY)
			enable_group(1);
	}
}

void trigger_soft_intr(unsigned int id)
{
	unsigned int val = gicd_read_itargetsr(get_plat_config()->gicd_base, id);

	gicd_write_itargetsr(get_plat_config()->gicd_base,
				id,
				val & ~(0xff << ((id % 4) * 8)));
	gicd_set_itargetsr(get_plat_config()->gicd_base,
				id,
				platform_get_core_pos(read_mpidr()));
	gicd_set_ispendr(get_plat_config()->gicd_base, id);
}

void sec_exc_add(unsigned int intr_num)
{
	sec_exc[SEC_EXC_CNT] = intr_num;
	SEC_EXC_CNT++;
}

void nsec_exc_add(unsigned int intr_num)
{
	nsec_exc[NSEC_EXC_CNT] = intr_num;
	NSEC_EXC_CNT++;
}

void teei_init_interrupt(void)
{
	int i;

	SEC_EXC_CNT = 0;
	NSEC_EXC_CNT = 0;

	teeiBootCfg = (tee_arg_t_ptr)(uintptr_t)TEEI_BOOT_PARAMS;
	teeiKeys = (tee_keys_t_ptr)(uintptr_t)TEEI_SECURE_PARAMS;

	for (i = 0; i < 5; i++) {
		if (teeiBootCfg->tee_dev[i].dev_type == MT_UART16550) {
			uart_apc_num = teeiBootCfg->tee_dev[i].apc_num;
			NSEC_UART = teeiBootCfg->tee_dev[i].intr_num;
		}
		if (teeiBootCfg->tee_dev[i].dev_type == MT_SEC_GPT)
			SEC_TIMER = teeiBootCfg->tee_dev[i].intr_num;
	}

	SEC_APP_INTR = teeiBootCfg->ssiq_number[0];
	SEC_DRV_INTR = teeiBootCfg->ssiq_number[1];
	SEC_RDRV_INTR = teeiBootCfg->ssiq_number[2];
	SEC_TDUMP_INTR = teeiBootCfg->ssiq_number[3];
	NSEC_TIMER = 30;

	/* init secure exlusive array */
	sec_exc_add(SEC_TIMER);
	sec_exc_add(SEC_APP_INTR);
	sec_exc_add(SEC_DRV_INTR);
	sec_exc_add(SEC_RDRV_INTR);
	sec_exc_add(SEC_TDUMP_INTR);
#ifdef SPI_DOMINATED_IN_TEE
	sec_exc_add(SEC_SPI0);
#endif
#ifdef TUI_SUPPORT
	sec_exc_add(206);
	sec_exc_add(SEC_TUI_CANCEL);
#endif
	/*init non secure exlusive array*/
	nsec_exc_add(NSEC_TIMER);
}

unsigned int teei_get_fp_id(void)
{
	if (teeiBootCfg->tee_dev[4].dev_type == 5)
		return teeiBootCfg->tee_dev[4].base_addr;

	return 0;
}

void teei_gic_setup(void)
{
	unsigned int gicd_base;

	gicd_base = get_plat_config()->gicd_base;

	teei_init_interrupt();

	/* Configure  secure interrupts now */
	for (unsigned int index = 0; index < SEC_EXC_CNT; index++) {
		/* set this interrupt to group 0 */
		gicd_clr_igroupr(gicd_base, sec_exc[index]);
		/* set this interrupt GIC_HIGHEST_SEC_PRIORITY */
		gicd_set_ipriorityr(gicd_base, sec_exc[index], GIC_HIGHEST_SEC_PRIORITY);
		gicd_set_itargetsr(gicd_base, sec_exc[index] /*set itarget to current cpu */,
					platform_get_core_pos(read_mpidr()));
		/* set gic edge sensitive via GICD_ICFG */
		mt_irq_set_sens(gicd_base, sec_exc[index], MT_EDGE_SENSITIVE);
		/* set low polarity */
		mt_irq_set_polarity(sec_exc[index], MT_POLARITY_LOW);
		/* disable  this interrupt in gicd */
		gicd_set_icenabler(gicd_base, sec_exc[index]);
#ifdef TUI_SUPPORT
		gicd_set_icpendr(gicd_base, 206);
#endif
	}
}

void plat_teei_dump(void)
{
	fiq_icc_isr();
}

void teei_triggerSgiDump(void)
{
	uint64_t mpidr;
	uint32_t linear_id;

	/* send to all cpus except the current one */
	mpidr = read_mpidr();
	linear_id = platform_get_core_pos(mpidr);
	fiq_smp_call_function(0xFF & ~(1 << linear_id), aee_wdt_dump, 0, 0);
	aee_wdt_dump();
}

void teei_ack_gic(void)
{
	uint32_t iar;

	iar = gicc_read_IAR(get_plat_config()->gicc_base);
	gicc_write_EOIR(get_plat_config()->gicc_base, iar);
}

uint32_t teei_rng(void)
{
	uint32_t status = 0;
	uint32_t value = 0;

	status = plat_get_rnd(&value);
	if (status != 0)
		value = 0xffffffff;

	return value;
}

/* fiq is handled by S-EL1 */
uint64_t teei_fiq_handler(uint32_t id,
				uint32_t flags,
				void *handle,
				void *cookie)
{
	uint64_t mpidr = read_mpidr();
	uint32_t linear_id = platform_get_core_pos(mpidr);
	teei_context *teei_ctx = &secure_context[linear_id];
	int caller_security_state = flags & 1;

	/* set_module_apc(uart_apc_num, 0, 1);*/ /* set uart secure */
	/* set_uart_flag(); */

	if (caller_security_state == SECURE)
		SMC_RET1(handle, SMC_UNK);

	switch (id) {
	case FIQ_SMP_CALL_SGI: {
		/* DBG_PRINTF("teei_fiq_handler : FIQ_SMP_CALL_SGI\n"); */
		plat_ic_acknowledge_interrupt();
		fiq_icc_isr();
		plat_ic_end_of_interrupt(FIQ_SMP_CALL_SGI);
		SMC_RET0(handle);
		break;
	}
	case WDT_IRQ_BIT_ID: {
		/* printf("teei_fiq_handler : Catch WDT FIQ !!!\n");*/
		/* if disable group 0 other core will not receive sgi */
		/* disable_group(0); */
		int lockval = 0;
		int tmp = 0;

		plat_ic_acknowledge_interrupt();

		__asm__ volatile(
			"1: ldxr  %w0, [%2]\n"
			" add %w0, %w0, %w3\n"
			" stxr  %w1, %w0, [%2]\n"
			" cbnz  %w1, 1b"
			: "=&r"(lockval),  "=&r"(tmp)
			: "r"(&(s_wdt_lock)), "Ir"(1)
			: "cc");

		if (s_wdt_lock == 1)
			teei_triggerSgiDump();


		plat_ic_end_of_interrupt(WDT_IRQ_BIT_ID);
		SMC_RET0(handle);
		break;
	}
	default: {
		/* DBG_PRINTF("teei_fiq_handler : UNK FIQ NUM\n"); */
		SMC_RET0(handle);
	}
	}

	return 0;
}

