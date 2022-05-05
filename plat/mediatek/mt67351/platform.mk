#
# Copyright (c) 2013-2014, ARM Limited and Contributors. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# Neither the name of ARM nor the names of its contributors may be used
# to endorse or promote products derived from this software without specific
# prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Shared memory may be allocated at the top of Trusted SRAM (tsram) or at the
# base of Trusted SRAM (tdram)

MTK_PLAT		:=	plat/mediatek
MTK_PLAT_SOC		:=	${MTK_PLAT}/${PLAT}

PLAT_INCLUDES := -I${MTK_PLAT_SOC}/include \
				-I${MTK_PLAT_SOC}/ \
				-I${MTK_PLAT_SOC}/drivers/timer \
				-I${MTK_PLAT_SOC}/drivers/devapc \
				-I${MTK_PLAT_SOC}/drivers/l2c \
				-I${MTK_PLAT_SOC}/drivers/emi \
				-I${MTK_PLAT_SOC}/drivers/pmic \
				-I${MTK_PLAT_SOC}/drivers/rtc \
				-I${MTK_PLAT_SOC}/drivers/wdt \
				-Iinclude/common \
				-Iinclude/drivers/io \
				-Iinclude/plat/common \
				-Iinclude/bl31/services \
				-Iinclude/bl31 \
				-Iinclude/drivers/arm \
				-Iinclude/drivers \
				-Iinclude/lib \
				-I${MTK_PLAT_SOC}/drivers/sec \
				-Idrivers/arm/gic/v2 \
				-Idrivers/arm/gic/common \
				-I${MTK_PLAT}/common

PLAT_BL_COMMON_SOURCES := drivers/io/io_fip.c \
				drivers/io/io_memmap.c \
				drivers/io/io_semihosting.c \
				drivers/io/io_storage.c \
				lib/xlat_tables/xlat_tables_common.c \
				lib/xlat_tables/aarch64/xlat_tables.c \
				lib/semihosting/semihosting.c \
				lib/semihosting/aarch64/semihosting_call.S \
				drivers/arm/pl011/aarch64/pl011_console.S \
				${MTK_PLAT_SOC}/plat_io_storage.c \
				plat/common/aarch64/crash_console_helpers.S


BL31_SOURCES		+=	drivers/arm/cci/cci.c \
				drivers/arm/gic/v2/gicv2_main.c \
				drivers/arm/gic/v3/gicv3_main.c \
				drivers/arm/tzc/tzc400.c \
				drivers/arm/gic/v2/gicdv2_helpers.c \
				drivers/arm/gic/v2/gicv2_helpers.c \
				plat/common/plat_psci_common.c \
				lib/cpus/aarch64/aem_generic.S \
				lib/cpus/aarch64/cortex_a53.S \
				lib/cpus/aarch64/cortex_a57.S \
				plat/common/aarch64/platform_mp_stack.S \
				${MTK_PLAT}/common/drivers/pmic_wrap/pmic_wrap_init.c	\
				${MTK_PLAT}/common/params_setup.c                     \
				${MTK_PLAT}/common/drivers/rtc/rtc_common.c	\
				${MTK_PLAT}/common/mtk_plat_common.c		\
				${MTK_PLAT}/common/mtk_sip_svc.c		\
				${MTK_PLAT_SOC}/aarch64/platform_common.c \
				drivers/delay_timer/delay_timer.c		\
				drivers/delay_timer/generic_delay_timer.c	\
				drivers/ti/uart/aarch64/16550_console.S		\
				${MTK_PLAT_SOC}/bl31_plat_setup.c \
				${MTK_PLAT_SOC}/plat_gic.c \
				${MTK_PLAT_SOC}/plat_pm.c \
				${MTK_PLAT_SOC}/plat_security.c \
				${MTK_PLAT_SOC}/plat_topology.c \
				${MTK_PLAT_SOC}/scu.c \
				${MTK_PLAT_SOC}/mailbox.c \
				${MTK_PLAT_SOC}/aarch64/plat_helpers.S \
				${MTK_PLAT_SOC}/drivers/timer/mt_cpuxgpt.c \
				${MTK_PLAT_SOC}/drivers/pwrc/plat_pwrc.c \
				${MTK_PLAT_SOC}/drivers/devapc/devapc.c \
				${MTK_PLAT_SOC}/drivers/l2c/l2c.c \
				${MTK_PLAT_SOC}/drivers/emi/emi_mpu.c \
				${MTK_PLAT_SOC}/drivers/sec/rng.c \
				${MTK_PLAT_SOC}/drivers/rtc/rtc.c \
				${MTK_PLAT_SOC}/drivers/wdt/wdt.c \



# Flag used by the MTK_platform port to determine the version of ARM GIC architecture
# to use for interrupt management in EL3.
MT_GIC_ARCH := 2
$(eval $(call add_define,MT_GIC_ARCH))

# Enable workarounds for selected Cortex-A53 erratas.
ERRATA_A53_826319       :=      1
ERRATA_A53_836870       :=      1
