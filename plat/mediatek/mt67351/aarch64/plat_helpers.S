/*
 * Copyright (c) 2013-2014, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <arch.h>
#include <asm_macros.S>
#include <bl_common.h>
#include <gicv2.h>
#include "../plat_def.h"
#include <pl011.h>
#include "../drivers/pwrc/plat_pwrc.h"

	.globl	plat_get_my_entrypoint
	.globl	plat_secondary_cold_boot_setup
	.globl	platform_mem_init
	.globl	plat_report_exception
	.globl	platform_is_primary_cpu
	.globl  plat_my_core_pos
	.globl	write_cpuectlr
	.globl  read_l2actlr
	.globl  write_l2actlr
	.globl  read_l2ectlr
	.globl  write_l2ectlr
	.globl  read_cpuactlr
	.globl  write_cpuactlr
        .globl	read_cpuectlr
	/* -----------------------------------------------------
	 * void plat_secondary_cold_boot_setup (void);
	 *
	 * This function performs any platform specific actions
	 * needed for a secondary cpu after a cold reset e.g
	 * mark the cpu's presence, mechanism to place it in a
	 * holding pen etc.
	 * TODO: Should we read the PSYS register to make sure
	 * that the request has gone through.
	 * -----------------------------------------------------
	 */
func plat_secondary_cold_boot_setup
#ifndef EL3_PAYLOAD_BASE
	/* ---------------------------------------------
	 * Power down this cpu.
	 * TODO: Do we need to worry about powering the
	 * cluster down as well here. That will need
	 * locks which we won't have unless an elf-
	 * loader zeroes out the zi section.
	 * ---------------------------------------------
	 */
	mrs	x0, mpidr_el1
	mov_imm	x1, PWRC_BASE
	str	w0, [x1, #PPOFFR_OFF]

	/* ---------------------------------------------
	 * There is no sane reason to come out of this
	 * wfi so panic if we do. This cpu will be pow-
	 * ered on and reset by the cpu_on pm api
	 * ---------------------------------------------
	 */
	dsb	sy
	wfi
	no_ret	plat_panic_handler
#else
	mov_imm	x0, PLAT_ARM_TRUSTED_MAILBOX_BASE

	/* Wait until the entrypoint gets populated */
poll_mailbox:
	ldr	x1, [x0]
	cbz	x1, 1f
	br	x1
1:
	wfe
	b	poll_mailbox
#endif
endfunc plat_secondary_cold_boot_setup
	/* -----------------------------------------------------
	 * void platform_get_entrypoint (unsigned int mpid);
	 *
	 * Main job of this routine is to distinguish between
	 * a cold and warm boot.
	 * On a cold boot the secondaries first wait for the
	 * platform to be initialized after which they are
	 * hotplugged in. The primary proceeds to perform the
	 * platform initialization.
	 * On a warm boot, each cpu jumps to the address in its
	 * mailbox.
	 *
	 * TODO: Not a good idea to save lr in a temp reg
	 * TODO: PSYSR is a common register and should be
	 * 	accessed using locks. Since its not possible
	 * 	to use locks immediately after a cold reset
	 * 	we are relying on the fact that after a cold
	 * 	reset all cpus will read the same WK field
	 * -----------------------------------------------------
	 */


func plat_get_my_entrypoint
	mov	x9, x30 // lr

        /*Check first_boot flag
	 if first_boot flag is not 0, do cold boot initialization,
	 then set first_boot flag to 0
	 else, it's warm boot, jump via mailbox
	*/
	ldr		x1, =first_boot
	ldr		w2, [x1]
	cmp		w2, #0
	beq		warm_reset
	str		wzr, [x1]
	mov		x0,	#0
	b		exit

warm_reset:
	/* ---------------------------------------------
	 * A per-cpu mailbox is maintained in the tru-
	 * sted DRAM. Its flushed out of the caches
	 * after every update using normal memory so
	 * its safe to read it here with SO attributes
	 * ---------------------------------------------
	 */
	ldr	x10, =MBOX_BASE
	bl	plat_my_core_pos
	lsl	x0, x0, #CACHE_WRITEBACK_SHIFT
	ldr	x0, [x10, x0]
	cbz	x0, _panic
exit:
	ret	x9
_panic:	b	_panic
endfunc plat_get_my_entrypoint

	/* -----------------------------------------------------
	 * void platform_mem_init (void);
	 *
	 * Zero out the mailbox registers in the shared memory.
	 * The mmu is turned off right now and only the primary can
	 * ever execute this code. Secondaries will read the
	 * mailboxes using SO accesses. In short, BL31 will
	 * update the mailboxes after mapping the tzdram as
	 * normal memory. It will flush its copy after update.
	 * BL1 will always read the mailboxes with the MMU off
	 * -----------------------------------------------------
	 */
func platform_mem_init
	ldr	x0, =MBOX_BASE
	mov	w1, #PLATFORM_CORE_COUNT
loop:
	str	xzr, [x0], #CACHE_WRITEBACK_GRANULE
	subs	w1, w1, #1
	b.gt	loop
	ret
endfunc platform_mem_init
	/* ---------------------------------------------
	 * void plat_report_exception(unsigned int type)
	 * Function to report an unhandled exception
	 * with platform-specific means.
	 * On FVP platform, it updates the LEDs
	 * to indicate where we are
	 * ---------------------------------------------
	 */
func plat_my_core_pos
	mrs     x0, mpidr_el1
	and     x1, x0, #MPIDR_CPU_MASK
	and     x0, x0, #MPIDR_CLUSTER_MASK
	add     x0, x1, x0, LSR #6
	ret
endfunc plat_my_core_pos

func read_l2actlr
        dsb sy
        MRS x0, S3_1_C15_C0_0     /* @Read L2ACTLR_EL1 into Rt */
        dsb sy
        isb
	ret
endfunc read_l2actlr

func write_l2actlr
        dsb sy
        MSR S3_1_C15_C0_0, x0     /* @write L2ACTLR_EL1 into Rt */
        isb
        dsb sy
	ret
endfunc write_l2actlr

func read_l2ectlr
        MRS x0, S3_1_C11_C0_3     /* @Read L2ECTLR_EL1 into Rt */
        dsb sy
        isb
	ret
endfunc read_l2ectlr

func write_l2ectlr
        dsb sy
        MSR S3_1_C11_C0_3, x0     /* @write L2ECTLR_EL1 into Rt */
        isb
        dsb sy
	ret
endfunc write_l2ectlr

func read_cpuactlr
        dsb sy
        MRS x0, S3_1_C15_C2_0     /* @Read CPUACTLR_EL1 into Rt */
        dsb sy
        isb
	ret
endfunc read_cpuactlr

func write_cpuactlr
        dsb sy
        MSR S3_1_C15_C2_0, x0     /* @write CPUACTLR_EL1 into Rt */
        isb
        dsb sy
	ret
endfunc write_cpuactlr

func read_cpuectlr
	MRS	x0, S3_1_C15_C2_1
	ret
endfunc read_cpuectlr

func write_cpuectlr
	MSR	S3_1_C15_C2_1, x0
	ret
endfunc write_cpuectlr

func platform_is_primary_cpu
	and	x0, x0, #(MPIDR_CLUSTER_MASK | MPIDR_CPU_MASK)
	cmp	x0, #FVP_PRIMARY_CPU
	cset	x0, eq
	ret
endfunc platform_is_primary_cpu
	/* Define a crash console for the plaform */
#define FVP_CRASH_CONSOLE_BASE		PL011_UART0_BASE

	/* ---------------------------------------------
	 * int plat_crash_console_init(void)
	 * Function to initialize the crash console
	 * without a C Runtime to print crash report.
	 * Clobber list : x0, x1, x2
	 * ---------------------------------------------
	 */

	.data
first_boot:
	.long	0x1


