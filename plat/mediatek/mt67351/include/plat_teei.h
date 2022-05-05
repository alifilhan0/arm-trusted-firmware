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

#ifndef __PLAT_TEEI_H__
#define __PLAT_TEEI_H__

#include "devapc.h"
#include <platform.h>
#include <plat_private.h>

#define INTR_OFFSET 0

#if 0
#define SEC_APP_INTR		(280+INTR_OFFSET)
#define SEC_DRV_INTR		(286+INTR_OFFSET)
#define SEC_RDRV_INTR		(279+INTR_OFFSET)
#define SEC_TDUMP_INTR		(282+INTR_OFFSET)
#else
extern unsigned int SEC_APP_INTR;
extern unsigned int SEC_DRV_INTR;
extern unsigned int SEC_RDRV_INTR;
extern unsigned int SEC_TDUMP_INTR;
#endif
extern unsigned int NSEC_UART;
extern uint32_t uart_apc_num;
extern uint32_t spi_apc_num;
extern uint32_t TEEI_STATE;

#define NSEC_CALL_INTR		(284+INTR_OFFSET)
#define NSEC_INTSCHED_INTR	(285+INTR_OFFSET)

#define NSEC_APP_INTR		282
#define NSEC_BOOT_INTR		283
#define NSEC_SCHED_INTR		284
#define NSEC_DRV_INTR		287
#define NSEC_RDRV_INTR		278
#define NSEC_LOG_INTR		277
#define NSEC_ERR_INTR		276
#define SEC_TUI_CANCEL		275

#define TEEI_BOOT_PARAMS (((atf_arg_t_ptr)(uintptr_t)TEE_BOOT_INFO_ADDR)->tee_boot_arg_addr)
/* Move key param from (base + 0x200) to (base + 0x300)
 * secure isram base            : atf param
 * secure isram base + 0x100    : teei boot param (TEEI_BOOT_PARAMS)
 * secure isram base + 0x200    : secure memory param
 * secure isram base + 0x300    : teei key param (TEEI_BOOT_PARAMS + 0x200)
 */
#define TEEI_SECURE_PARAMS (TEEI_BOOT_PARAMS+0x200)

/* this should be checked and changed by platform */
#if defined(MACH_TYPE_MT6735) || defined(MACH_TYPE_MT6753)
#define TEEI_SPI0_APC_NUM	(54)
#elif defined(MACH_TYPE_MT6735M)
#define TEEI_SPI0_APC_NUM	(53)
#else
#error "Wrong MACH Type!!"
#endif
#define TEEI_USED_SPI_NUM	TEEI_SPI0_APC_NUM

void migrate_gic_context(uint32_t secure_state);
void disable_group(unsigned int grp);
void enable_group(unsigned int grp);
void trigger_soft_intr(unsigned int id);
void prepare_gic_for_nsec_boot(void);
void prepare_gic_for_sec_boot(void);
void teei_gic_setup(void);
void teei_ack_gic(void);
void teei_triggerSgiDump(void);
void plat_teei_dump(void);

extern void mt_irq_set_sens(unsigned int base, unsigned int irq, unsigned int sens);
extern void mt_irq_set_polarity(unsigned int irq, unsigned int polarity);
extern void set_module_apc(unsigned int module, E_MASK_DOM domain_num , APC_ATTR permission_control);
extern void tf_printf(const char *fmt, ...);
extern void mt_log_secure_os_print(int c);
extern unsigned int get_irq_target(unsigned int irq);
extern void aee_wdt_dump(void);
extern void set_module_apc(unsigned int module, E_MASK_DOM domain_num,
		APC_ATTR permission_control);
extern uint64_t teei_fiq_handler(uint32_t id, uint32_t flags, void *handle,
		void *cookie);
extern void gicd_v3_do_wait_for_rwp(unsigned int gicd_base);
extern int gic_populate_rdist(unsigned int *rdist_base);
extern uint64_t gic_mpidr_to_affinity(uint64_t mpidr);
extern void aee_wdt_dump(void);
#ifndef LEGACY_GIC_SUPPORT
extern void gicc_write_grpen1_el1(uint64_t val);
#endif

#endif
