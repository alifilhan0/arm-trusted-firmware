/*
 * Copyright (c) 2019, MediaTek Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <lib/mmio.h>
#include <mtcmos.h>
#include <platform_def.h>
#include <spm.h>

#define SPM_EN			(0xb16 << 16 | 0x1)
#define SPM_VDE_PWR_CON		0x0210
#define SPM_MFG_PWR_CON		0x0214
#define SPM_VEN_PWR_CON			0x0230
#define SPM_ISP_PWR_CON		0x0238
#define SPM_DIS_PWR_CON		0x023c
#define SPM_CONN_PWR_CON	0x0280

#define SRAM_ISOINT_B           BIT(6)
#define SRAM_CKISO              BIT(5)
#define L1_PDN			BIT(0)
#define L1_PDN_ACK		BIT(8)
#define PWR_RST_B		BIT(0)
#define PWR_ISO			BIT(1)
#define PWR_ON			BIT(2)
#define PWR_ON_2ND		BIT(3)
#define PWR_CLK_DIS		BIT(4)

#define PWR_STATUS_MD1		BIT(0)
#define PWR_STATUS_CONN		BIT(1)
#define PWR_STATUS_DISP		BIT(3)
#define PWR_STATUS_MFG		BIT(4)
#define PWR_STATUS_ISP		BIT(5)
#define PWR_STATUS_VDEC		BIT(7)
#define PWR_STATUS_VENC		BIT(8)
#define PWR_STATUS_MD2		BIT(22)

/* Infrasys configuration */
#define INFRA_TOPDCM_CTRL	0x10
#define INFRA_TOPAXI_PROT_EN	0x220
#define INFRA_TOPAXI_PROT_STA1	0x228
#define INFRA_TOPAXI_PROT_STA1  0x228

struct scp_domain_data {
	uint32_t sta_mask;
	int ctl_offs;
	uint32_t sram_pdn_bits;
	uint32_t sram_pdn_ack_bits;
	uint32_t bus_prot_mask;
};

enum {
	LITTLE_CPU3	= 1U << 12,
	LITTLE_CPU2	= 1U << 11,
	LITTLE_CPU1	= 1U << 10,
};

static struct scp_domain_data scp_domain_mt6735[] = {
	[MT6735_POWER_DOMAIN_CONN] = {
		.sta_mask = PWR_STATUS_CONN,
		.ctl_offs = SPM_CONN_PWR_CON,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = 0,
	},
	[MT6735_POWER_DOMAIN_VENC] = {
		.sta_mask = PWR_STATUS_VENC,
		.ctl_offs = SPM_VEN_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(15, 12),
	},

	[MT6735_POWER_DOMAIN_DISP] = {
		.sta_mask = PWR_STATUS_DISP,
		.ctl_offs = SPM_DIS_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
	},
	[MT6735_POWER_DOMAIN_MFG] = {
		.sta_mask = PWR_STATUS_MFG,
		.ctl_offs = SPM_MFG_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
	},
	[MT6735_POWER_DOMAIN_VDEC] = {
		.sta_mask = PWR_STATUS_VDEC,
		.ctl_offs = SPM_VDE_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(12, 12),
	},
	[MT6735_POWER_DOMAIN_ISP] = {
		.sta_mask = PWR_STATUS_ISP,
		.ctl_offs = SPM_ISP_PWR_CON,
		.sram_pdn_bits = GENMASK(11, 8),
		.sram_pdn_ack_bits = GENMASK(13, 12),
	},
	[MT6735_POWER_DOMAIN_MD1] = {
		.sta_mask = PWR_STATUS_MD1,
		.ctl_offs = 0x0284,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = 0,
	},
	[MT6735_POWER_DOMAIN_MD2] = {
		.sta_mask = PWR_STATUS_MD2,
		.ctl_offs = 0x02d4,
		.sram_pdn_bits = GENMASK(8, 8),
		.sram_pdn_ack_bits = 0,
    },
};

static void mtcmos_ctrl_little_on(unsigned int linear_id)
{
	uint32_t reg_pwr_con;
	uint32_t reg_l1_pdn;
	uint32_t bit_cpu;

	switch (linear_id) {
	case 1:
		reg_pwr_con = SPM_CA7_CPU1_PWR_CON;
		reg_l1_pdn = SPM_CA7_CPU1_L1_PDN;
		bit_cpu = LITTLE_CPU1;
		break;
	case 2:
		reg_pwr_con = SPM_CA7_CPU2_PWR_CON;
		reg_l1_pdn = SPM_CA7_CPU2_L1_PDN;
		bit_cpu = LITTLE_CPU2;
		break;
	case 3:
		reg_pwr_con = SPM_CA7_CPU3_PWR_CON;
		reg_l1_pdn = SPM_CA7_CPU3_L1_PDN;
		bit_cpu = LITTLE_CPU3;
		break;
	default:
		/* should never come to here */
		return;
	}
	/* enable register control */
	mmio_write_32(SPM_POWERON_CONFIG_SET,
		      SPM_REGWR_CFG_KEY | SPM_REGWR_EN);

	mmio_setbits_32(reg_pwr_con, PWR_ON);
	udelay(1);
	mmio_setbits_32(reg_pwr_con, PWR_ON_2ND);

	while ((mmio_read_32(SPM_PWR_STATUS) & bit_cpu) != bit_cpu ||
	       (mmio_read_32(SPM_PWR_STATUS_2ND) & bit_cpu) != bit_cpu)
		continue;

	mmio_clrbits_32(reg_pwr_con, PWR_ISO);
	mmio_clrbits_32(reg_l1_pdn, L1_PDN);

	while (mmio_read_32(reg_l1_pdn) & L1_PDN_ACK)
		continue;

	mmio_setbits_32(reg_pwr_con, SRAM_ISOINT_B);
	mmio_clrbits_32(reg_pwr_con, SRAM_CKISO);
	mmio_clrbits_32(reg_pwr_con, PWR_CLK_DIS);
	mmio_setbits_32(reg_pwr_con, PWR_RST_B);
}

void mtcmos_little_cpu_on(void)
{
	mtcmos_ctrl_little_on(1);
	mtcmos_ctrl_little_on(2);
	mtcmos_ctrl_little_on(3);
}

static void mtcmos_ctrl_little_off(unsigned int linear_id)
{
	uint32_t reg_pwr_con;
	uint32_t reg_l1_pdn;
	uint32_t bit_cpu;

	switch (linear_id) {
	case 1:
		reg_pwr_con = SPM_CA7_CPU1_PWR_CON;
		reg_l1_pdn = SPM_CA7_CPU1_L1_PDN;
		bit_cpu = LITTLE_CPU1;
		break;
	case 2:
		reg_pwr_con = SPM_CA7_CPU2_PWR_CON;
		reg_l1_pdn = SPM_CA7_CPU2_L1_PDN;
		bit_cpu = LITTLE_CPU2;
		break;
	case 3:
		reg_pwr_con = SPM_CA7_CPU3_PWR_CON;
		reg_l1_pdn = SPM_CA7_CPU3_L1_PDN;
		bit_cpu = LITTLE_CPU3;
		break;
	default:
		/* should never come to here */
		return;
	}

	/* enable register control */
	mmio_write_32(SPM_POWERON_CONFIG_SET,
			(SPM_PROJECT_CODE << 16) | (1U << 0));

	mmio_setbits_32(reg_pwr_con, PWR_ISO);
	mmio_setbits_32(reg_pwr_con, SRAM_CKISO);
	mmio_clrbits_32(reg_pwr_con, SRAM_ISOINT_B);
	mmio_setbits_32(reg_l1_pdn, L1_PDN);

	while (!(mmio_read_32(reg_l1_pdn) & L1_PDN_ACK))
		continue;

	mmio_clrbits_32(reg_pwr_con, PWR_RST_B);
	mmio_setbits_32(reg_pwr_con, PWR_CLK_DIS);
	mmio_clrbits_32(reg_pwr_con, PWR_ON);
	mmio_clrbits_32(reg_pwr_con, PWR_ON_2ND);

	while ((mmio_read_32(SPM_PWR_STATUS) & bit_cpu) ||
	       (mmio_read_32(SPM_PWR_STATUS_2ND) & bit_cpu))
		continue;
}

void mtcmos_little_cpu_off(void)
{
	/* turn off little cpu 1 - 3 */
	mtcmos_ctrl_little_off(1);
	mtcmos_ctrl_little_off(2);
	mtcmos_ctrl_little_off(3);
}

static int mtcmos_power_on(uint32_t on, struct scp_domain_data *data)
{
	uintptr_t ctl_addr = SPM_BASE + data->ctl_offs;
	uint32_t val, pdn_ack = data->sram_pdn_ack_bits;

	mmio_write_32(SPM_POWERON_CONFIG_SET,
		      SPM_REGWR_CFG_KEY | SPM_REGWR_EN);

	val = mmio_read_32(ctl_addr);
	val |= PWR_ON;
	mmio_write_32(ctl_addr, val);

	val |= PWR_ON_2ND;
	mmio_write_32(ctl_addr, val);

	while (!(mmio_read_32(SPM_PWR_STATUS) & data->sta_mask) ||
	       !(mmio_read_32(SPM_PWR_STATUS_2ND) & data->sta_mask))
		continue;

	val &= ~PWR_CLK_DIS;
	mmio_write_32(ctl_addr, val);

	val &= ~PWR_ISO;
	mmio_write_32(ctl_addr, val);

	val |= PWR_RST_B;
	mmio_write_32(ctl_addr, val);

	val &= ~data->sram_pdn_bits;
	mmio_write_32(ctl_addr, val);

	while ((mmio_read_32(ctl_addr) & pdn_ack))
		continue;

	if (data->bus_prot_mask) {
		mmio_clrbits_32(INFRACFG_AO_BASE + INFRA_TOPAXI_PROT_EN,
				data->bus_prot_mask);
		while ((mmio_read_32(INFRACFG_AO_BASE + INFRA_TOPAXI_PROT_STA1) &
			data->bus_prot_mask))
			continue;
	}

	return 0;
}

int mtcmos_non_cpu_ctrl(uint32_t on, uint32_t num)
{
	uint32_t ret = -1;
	struct scp_domain_data *data;

	switch (num) {
	case MT6735_POWER_DOMAIN_CONN:
		data = &scp_domain_mt6735[MT6735_POWER_DOMAIN_CONN];
		ret = mtcmos_power_on(on, data);
		break;
	case MT6735_POWER_DOMAIN_VENC:
		data = &scp_domain_mt6735[MT6735_POWER_DOMAIN_VENC];
		ret = mtcmos_power_on(on, data);
		break;
	case MT6735_POWER_DOMAIN_DISP:
		data = &scp_domain_mt6735[MT6735_POWER_DOMAIN_DISP];
		ret = mtcmos_power_on(on, data);
		break;
	case MT6735_POWER_DOMAIN_MFG:
		data = &scp_domain_mt6735[MT6735_POWER_DOMAIN_MFG];
		ret = mtcmos_power_on(on, data);
		break;
	case MT6735_POWER_DOMAIN_VDEC:
		data = &scp_domain_mt6735[MT6735_POWER_DOMAIN_VDEC];
		ret = mtcmos_power_on(on, data);
		break;
	case MT6735_POWER_DOMAIN_ISP:
		data = &scp_domain_mt6735[MT6735_POWER_DOMAIN_ISP];
		ret = mtcmos_power_on(on, data);
		break;
	case MT6735_POWER_DOMAIN_MD1:
		data = &scp_domain_mt6735[MT6735_POWER_DOMAIN_MD1];
		ret = mtcmos_power_on(on, data);
		break;
	case MT6735_POWER_DOMAIN_MD2:
		data = &scp_domain_mt6735[MT6735_POWER_DOMAIN_MD2];
		ret = mtcmos_power_on(on, data);
		break;
	default:
		INFO("No mapping MTCMOS(%d), ret = %d\n", num, ret);
		break;
	}

	return ret;
}
