/*
 * Copyright (c) 2014-2017, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef MCUCFG_H
#define MCUCFG_H

#include <stdint.h>

#include <mt6735_def.h>

struct mt6735_mcucfg_regs {
	uint32_t mp0_ca7l_cache_config; /* 0x0 */
	uint32_t mp0_cpu0_mem_delsel;	/* 0x4 */
	uint32_t mp0_cpu1_mem_delsel;	/* 0x8 */
	uint32_t mp0_cpu2_mem_delsel;	/* 0xc */
	uint32_t mp0_cpu3_mem_delsel;	/* 0x10 */
	uint32_t mp0_cache_mem_delsel;	/* 0x14 */
	uint32_t reserved1[5];		/* 0x18-0x28 */
	uint32_t mp0_axi_config;	/* 0x2c */
	uint32_t mp0_misc_config[10];	/* 0x30-0x54 */
	uint32_t mp0_ca7l_cfg_dis;	/* 0x58 */
	uint32_t mp0_ca7l_clken_ctrl;	/* 0x5c */
	uint32_t mp0_ca7l_rst_ctrl;	/* 0x60 */
	uint32_t mp0_ca7l_misc_config;	/* 0x64 */
	uint32_t aclken_div;		/* 0x68 */
	uint32_t pclken_div;		/* 0x6c */
	uint32_t mp0_mem_pwr_ctrl;	/* 0x70 */
	uint32_t mp0_armpll_div_ctrl;	/* 0x74 */
	uint32_t mp0_rst_status;	/* 0x78 */
	uint32_t mp0_dbg_ctrl;		/* 0x7c */
	uint32_t mp0_dbg_flag;		/* 0x80 */
	uint32_t dbg_pwr_ctrl;		/* 0x84 */
	uint32_t mp0_rw_rsvd0;		/* 0x88 */
	uint32_t mp0_rw_rsvd1;		/* 0x8c */
	uint32_t mp0_ro_rsvd;		/* 0x90 */
	uint32_t reserved2[3];		/* 0x94-0x9c */
	uint32_t int_pol_ctl[8];	/* 0x100-0x11c */
	uint32_t reserved[32];		/* 0x120-0x19c */
	uint32_t ap_bank4_map[2];	/* 0x200-0x204 */
	uint32_t bus_sync_sel;		/* 0x208 */
	uint32_t mp0_ca7l_ir_mon;	/* 0x20c */
	uint32_t reserved3[92];		/* 0x210-0x39c */
	uint32_t dfd_ctrl;		/* 0x400 */
	uint32_t dfd_cnt_l;		/* 0x404 */
	uint32_t dfd_cnt_h;		/* 0x408 */
	uint32_t reserved4[108];	/* 0x40c-0x66c */
	uint32_t xgpt_ctl;		/* 0x670 */
	uint32_t xgpt_idx;		/* 0x674 */
};

static struct mt6735_mcucfg_regs *const mt6735_mcucfg = (void *)MCUCFG_BASE;

/* cpu boot mode */
#define	MP0_CPUCFG_64BIT_SHIFT	12
#define	MP1_CPUCFG_64BIT_SHIFT	28
#define	MP0_CPUCFG_64BIT	(U(0xf) << MP0_CPUCFG_64BIT_SHIFT)
#define	MP1_CPUCFG_64BIT	(U(0xf) << MP1_CPUCFG_64BIT_SHIFT)

/* scu related */
enum {
	MP0_ACINACTM_SHIFT = 4,
	MP1_ACINACTM_SHIFT = 0,
	MP0_ACINACTM = 1 << MP0_ACINACTM_SHIFT,
	MP1_ACINACTM = 1 << MP1_ACINACTM_SHIFT
};

enum {
	MP1_DIS_RGU0_WAIT_PD_CPUS_L1_ACK_SHIFT = 0,
	MP1_DIS_RGU1_WAIT_PD_CPUS_L1_ACK_SHIFT = 4,
	MP1_DIS_RGU2_WAIT_PD_CPUS_L1_ACK_SHIFT = 8,
	MP1_DIS_RGU3_WAIT_PD_CPUS_L1_ACK_SHIFT = 12,
	MP1_DIS_RGU_NOCPU_WAIT_PD_CPUS_L1_ACK_SHIFT = 16,

	MP1_DIS_RGU0_WAIT_PD_CPUS_L1_ACK =
		0xf << MP1_DIS_RGU0_WAIT_PD_CPUS_L1_ACK_SHIFT,
	MP1_DIS_RGU1_WAIT_PD_CPUS_L1_ACK =
		0xf << MP1_DIS_RGU1_WAIT_PD_CPUS_L1_ACK_SHIFT,
	MP1_DIS_RGU2_WAIT_PD_CPUS_L1_ACK =
		0xf << MP1_DIS_RGU2_WAIT_PD_CPUS_L1_ACK_SHIFT,
	MP1_DIS_RGU3_WAIT_PD_CPUS_L1_ACK =
		0xf << MP1_DIS_RGU3_WAIT_PD_CPUS_L1_ACK_SHIFT,
	MP1_DIS_RGU_NOCPU_WAIT_PD_CPUS_L1_ACK =
		0xf << MP1_DIS_RGU_NOCPU_WAIT_PD_CPUS_L1_ACK_SHIFT
};

enum {
	MP1_AINACTS_SHIFT = 4,
	MP1_AINACTS = 1 << MP1_AINACTS_SHIFT
};

enum {
	MP1_SW_CG_GEN_SHIFT = 12,
	MP1_SW_CG_GEN = 1 << MP1_SW_CG_GEN_SHIFT
};

enum {
	MP1_L2RSTDISABLE_SHIFT = 14,
	MP1_L2RSTDISABLE = 1 << MP1_L2RSTDISABLE_SHIFT
};

/* cci clock control related */
enum {
	MCU_BUS_DCM_EN	= 1 << 8
};

/* l2c sram control related */
enum {
	L2C_SRAM_DCM_EN = 1 << 0
};

/* bus fabric dcm control related */
enum {
	PSYS_ADB400_DCM_EN		= 1 << 29,
	GPU_ADB400_DCM_EN		= 1 << 28,

	EMI1_ADB400_DCM_EN		= 1 << 27,
	EMI_ADB400_DCM_EN		= 1 << 26,
	INFRA_ADB400_DCM_EN		= 1 << 25,
	L2C_ADB400_DCM_EN		= 1 << 24,

	MP0_ADB400_DCM_EN		= 1 << 23,
	CCI400_CK_ONLY_DCM_EN		= 1 << 22,
	L2C_IDLE_DCM_EN			= 1 << 21,

	CA15U_ADB_DYNAMIC_CG_EN		= 1 << 19,
	CA7L_ADB_DYNAMIC_CG_EN		= 1 << 18,
	L2C_ADB_DYNAMIC_CG_EN		= 1 << 17,

	EMICLK_EMI1_DYNAMIC_CG_EN	= 1 << 12,

	INFRACLK_PSYS_DYNAMIC_CG_EN	= 1 << 11,
	EMICLK_GPU_DYNAMIC_CG_EN	= 1 << 10,
	EMICLK_EMI_DYNAMIC_CG_EN	= 1 << 8,

	CCI400_SLV_RW_DCM_EN		= 1 << 7,
	CCI400_SLV_DCM_EN		= 1 << 5,

	ACLK_PSYS_DYNAMIC_CG_EN		= 1 << 3,
	ACLK_GPU_DYNAMIC_CG_EN		= 1 << 2,
	ACLK_EMI_DYNAMIC_CG_EN		= 1 << 1,
	ACLK_INFRA_DYNAMIC_CG_EN	= 1 << 0,

	/* adb400 related */
	ADB400_GRP_DCM_EN = PSYS_ADB400_DCM_EN | GPU_ADB400_DCM_EN |
			    EMI1_ADB400_DCM_EN | EMI_ADB400_DCM_EN |
			    INFRA_ADB400_DCM_EN | L2C_ADB400_DCM_EN |
			    MP0_ADB400_DCM_EN,

	/* cci400 related */
	CCI400_GRP_DCM_EN = CCI400_CK_ONLY_DCM_EN | CCI400_SLV_RW_DCM_EN |
			    CCI400_SLV_DCM_EN,

	/* adb clock related */
	ADBCLK_GRP_DCM_EN = CA15U_ADB_DYNAMIC_CG_EN | CA7L_ADB_DYNAMIC_CG_EN |
			    L2C_ADB_DYNAMIC_CG_EN,

	/* emi clock related */
	EMICLK_GRP_DCM_EN = EMICLK_EMI1_DYNAMIC_CG_EN |
			    EMICLK_GPU_DYNAMIC_CG_EN |
			    EMICLK_EMI_DYNAMIC_CG_EN,

	/* bus clock related */
	ACLK_GRP_DCM_EN = ACLK_PSYS_DYNAMIC_CG_EN | ACLK_GPU_DYNAMIC_CG_EN |
			  ACLK_EMI_DYNAMIC_CG_EN | ACLK_INFRA_DYNAMIC_CG_EN,
};

#endif /* MCUCFG_H */
