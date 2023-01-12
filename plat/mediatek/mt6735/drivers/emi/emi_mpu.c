//SPDX-License-Identifier: BSD-3-Clause
#include <assert.h>
#include <arch_helpers.h>
#include <stdio.h>
#include <common/debug.h>
#include <stdint.h>
#include <lib/mmio.h>
#include <plat_private.h>
#include <platform_def.h>
#include "emi_drv.h"


#define IOMEM(reg) (reg)
/*
 * emi_mpu_set_region_protection: protect a region.
 * @start: start address of the region
 * @end: end address of the region
 * @region: EMI MPU region id
 * @access_permission: EMI MPU access permission
 * Return 0 for success, otherwise negative status code.
 */
int emi_mpu_set_region_protection(unsigned int start, unsigned int end, int region, unsigned int access_permission)
{
	int ret = 0;
	unsigned int tmp, tmp2;
	unsigned int ax_pm, ax_pm2;

	if((end != 0) || (start !=0))
	{
		/*Address 64KB alignment*/
		start -= EMI_PHY_OFFSET;
		end -= EMI_PHY_OFFSET;
		start = start >> 16;
		end = end >> 16;

		if (end <= start)
		{
			return -1;
		}
	}

	ax_pm  = (access_permission << 16) >> 16;
	ax_pm2 = (access_permission >> 16);

	switch (region) {
	case 0:
		tmp = mmio_read_32(IOMEM(EMI_MPUI)) & 0xFFFF0000;
		tmp2 = mmio_read_32(IOMEM(EMI_MPUI_2ND)) & 0xFFFF0000;
		mmio_write_32(0, EMI_MPUI);
		mmio_write_32(0, EMI_MPUI_2ND);
		mmio_write_32((start << 16) | end, EMI_MPUA);
		mmio_write_32(tmp2 | ax_pm2, EMI_MPUI_2ND);
		mmio_write_32(tmp | ax_pm, EMI_MPUI);
		break;

	case 1:
		tmp = mmio_read_32(IOMEM(EMI_MPUI)) & 0x0000FFFF;
		tmp2 = mmio_read_32(IOMEM(EMI_MPUI_2ND)) & 0x0000FFFF;
		mmio_write_32(0, EMI_MPUI);
		mmio_write_32(0, EMI_MPUI_2ND);
		mmio_write_32((start << 16) | end, EMI_MPUB);
		mmio_write_32(tmp2 | (ax_pm2 << 16), EMI_MPUI_2ND);
		mmio_write_32(tmp | (ax_pm << 16), EMI_MPUI);
		break;

	case 2:
		tmp = mmio_read_32(IOMEM(EMI_MPUJ)) & 0xFFFF0000;
		tmp2 = mmio_read_32(IOMEM(EMI_MPUJ_2ND)) & 0xFFFF0000;
		mmio_write_32(0, EMI_MPUJ);
		mmio_write_32(0, EMI_MPUJ_2ND);
		mmio_write_32((start << 16) | end, EMI_MPUC);
		mmio_write_32(tmp2 | ax_pm2, EMI_MPUJ_2ND);
		mmio_write_32(tmp | ax_pm, EMI_MPUJ);
		break;

	case 3:
		tmp = mmio_read_32(IOMEM(EMI_MPUJ)) & 0x0000FFFF;
		tmp2 = mmio_read_32(IOMEM(EMI_MPUJ_2ND)) & 0x0000FFFF;
		mmio_write_32(0, EMI_MPUJ);
		mmio_write_32(0, EMI_MPUJ_2ND);
		mmio_write_32((start << 16) | end, EMI_MPUD);
		mmio_write_32(tmp2 | (ax_pm2 << 16), EMI_MPUJ_2ND);
		mmio_write_32(tmp | (ax_pm << 16), EMI_MPUJ);
		break;

	case 4:
		tmp = mmio_read_32(IOMEM(EMI_MPUK)) & 0xFFFF0000;
		tmp2 = mmio_read_32(IOMEM(EMI_MPUK_2ND)) & 0xFFFF0000;
		mmio_write_32(0, EMI_MPUK);
		mmio_write_32(0, EMI_MPUK_2ND);
		mmio_write_32((start << 16) | end, EMI_MPUE);
		mmio_write_32(tmp2 | ax_pm2, EMI_MPUK_2ND);
		mmio_write_32(tmp | ax_pm, EMI_MPUK);
		break;

	case 5:
		tmp = mmio_read_32(IOMEM(EMI_MPUK)) & 0x0000FFFF;
		tmp2 = mmio_read_32(IOMEM(EMI_MPUK_2ND)) & 0x0000FFFF;
		mmio_write_32(0, EMI_MPUK);
		mmio_write_32(0, EMI_MPUK_2ND);
		mmio_write_32((start << 16) | end, EMI_MPUF);
		mmio_write_32(tmp2 | (ax_pm2 << 16), EMI_MPUK_2ND);
		mmio_write_32(tmp | (ax_pm << 16), EMI_MPUK);
		break;

	case 6:
		tmp = mmio_read_32(IOMEM(EMI_MPUL)) & 0xFFFF0000;
		tmp2 = mmio_read_32(IOMEM(EMI_MPUL_2ND)) & 0xFFFF0000;
		mmio_write_32(0, EMI_MPUL);
		mmio_write_32(0, EMI_MPUL_2ND);
		mmio_write_32((start << 16) | end, EMI_MPUG);
		mmio_write_32(tmp2 | ax_pm2, EMI_MPUL_2ND);
		mmio_write_32(tmp | ax_pm, EMI_MPUL);
		break;

	case 7:
		tmp = mmio_read_32(IOMEM(EMI_MPUL)) & 0x0000FFFF;
		tmp2 = mmio_read_32(IOMEM(EMI_MPUL_2ND)) & 0x0000FFFF;
		mmio_write_32(0, EMI_MPUL);
		mmio_write_32(0, EMI_MPUL_2ND);
		mmio_write_32((start << 16) | end, EMI_MPUH);
		mmio_write_32(tmp2 | (ax_pm2 << 16), EMI_MPUL_2ND);
		mmio_write_32(tmp | (ax_pm << 16), EMI_MPUL);
		break;

	case 8:
		tmp = mmio_read_32(IOMEM(EMI_MPUI2)) & 0xFFFF0000;
		tmp2 = mmio_read_32(IOMEM(EMI_MPUI2_2ND)) & 0xFFFF0000;
		mmio_write_32(0, EMI_MPUI2);
		mmio_write_32(0, EMI_MPUI2_2ND);
		mmio_write_32((start << 16) | end, EMI_MPUA2);
		mmio_write_32(tmp2 | ax_pm2, EMI_MPUI2_2ND);
		mmio_write_32(tmp | ax_pm, EMI_MPUI2);
		break;

	case 9:
		tmp = mmio_read_32(IOMEM(EMI_MPUI2)) & 0x0000FFFF;
		tmp2 = mmio_read_32(IOMEM(EMI_MPUI2_2ND)) & 0x0000FFFF;
		mmio_write_32(0, EMI_MPUI2);
		mmio_write_32(0, EMI_MPUI2_2ND);
		mmio_write_32((start << 16) | end, EMI_MPUB2);
		mmio_write_32(tmp2 | (ax_pm2 << 16), EMI_MPUI2_2ND);
		mmio_write_32(tmp | (ax_pm << 16), EMI_MPUI2);
		break;

	case 10:
		tmp = mmio_read_32(IOMEM(EMI_MPUJ2)) & 0xFFFF0000;
		tmp2 = mmio_read_32(IOMEM(EMI_MPUJ2_2ND)) & 0xFFFF0000;
		mmio_write_32(0, EMI_MPUJ2);
		mmio_write_32(0, EMI_MPUJ2_2ND);
		mmio_write_32((start << 16) | end, EMI_MPUC2);
		mmio_write_32(tmp2 | ax_pm2, EMI_MPUJ2_2ND);
		mmio_write_32(tmp | ax_pm, EMI_MPUJ2);
		break;

	case 11:
		tmp = mmio_read_32(IOMEM(EMI_MPUJ2)) & 0x0000FFFF;
		tmp2 = mmio_read_32(IOMEM(EMI_MPUJ2_2ND)) & 0x0000FFFF;
		mmio_write_32(0, EMI_MPUJ2);
		mmio_write_32(0, EMI_MPUJ2_2ND);
		mmio_write_32((start << 16) | end, EMI_MPUD2);
		mmio_write_32(tmp2 | (ax_pm2 << 16), EMI_MPUJ2_2ND);
		mmio_write_32(tmp | (ax_pm << 16), EMI_MPUJ2);
		break;

	case 12:
		tmp = mmio_read_32(IOMEM(EMI_MPUK2)) & 0xFFFF0000;
		tmp2 = mmio_read_32(IOMEM(EMI_MPUK2_2ND)) & 0xFFFF0000;
		mmio_write_32(0, EMI_MPUK2);
		mmio_write_32(0, EMI_MPUK2_2ND);
		mmio_write_32((start << 16) | end, EMI_MPUE2);
		mmio_write_32(tmp2 | ax_pm2, EMI_MPUK2_2ND);
		mmio_write_32(tmp | ax_pm, EMI_MPUK2);
		break;

	case 13:
		tmp = mmio_read_32(IOMEM(EMI_MPUK2)) & 0x0000FFFF;
		tmp2 = mmio_read_32(IOMEM(EMI_MPUK2_2ND)) & 0x0000FFFF;
		mmio_write_32(0, EMI_MPUK2);
		mmio_write_32(0, EMI_MPUK2_2ND);
		mmio_write_32((start << 16) | end, EMI_MPUF2);
		mmio_write_32(tmp2 | (ax_pm2 << 16), EMI_MPUK2_2ND);
		mmio_write_32(tmp | (ax_pm << 16), EMI_MPUK2);
		break;

	case 14:
		tmp = mmio_read_32(IOMEM(EMI_MPUL2)) & 0xFFFF0000;
		tmp2 = mmio_read_32(IOMEM(EMI_MPUL2_2ND)) & 0xFFFF0000;
		mmio_write_32(0, EMI_MPUL2);
		mmio_write_32(0, EMI_MPUL2_2ND);
		mmio_write_32((start << 16) | end, EMI_MPUG2);
		mmio_write_32(tmp2 | ax_pm2, EMI_MPUL2_2ND);
		mmio_write_32(tmp | ax_pm, EMI_MPUL2);
		break;

	case 15:
		tmp = mmio_read_32(IOMEM(EMI_MPUL2)) & 0x0000FFFF;
		tmp2 = mmio_read_32(IOMEM(EMI_MPUL2_2ND)) & 0x0000FFFF;
		mmio_write_32(0, EMI_MPUL2);
		mmio_write_32(0, EMI_MPUL2_2ND);
		mmio_write_32((start << 16) | end, EMI_MPUH2);
		mmio_write_32(tmp2 | (ax_pm2 << 16), EMI_MPUL2_2ND);
		mmio_write_32(tmp | (ax_pm << 16), EMI_MPUL2);
		break;


	default:
		ret = -1;
		break;
	}

	return ret;
}




void emimpu_set_domain_secure_access(int domain, int sec_access)
{
	unsigned int d_ctrl_offset[]= {
		EMI_MPUM,
		EMI_MPUN,
		EMI_MPUO,
		EMI_MPUU,
		EMI_MPUM2,
		EMI_MPUN2,
		EMI_MPUO2,
		EMI_MPUU2,

	};
	unsigned int addr, value;

	if(domain < EMIMPU_DOMAIN_NUM)
	{
		addr = EMI_BASE + d_ctrl_offset[domain];
		value = mmio_read_32(addr);

		if(sec_access)
			value |= (0x1 << EMIMPU_DX_SEC_BIT);
		else
			value &= ~(0x1 << EMIMPU_DX_SEC_BIT);
	}

}

void emimpu_set_security_access(void)
{
	int domain;

	for(domain = 0; domain < EMIMPU_DOMAIN_NUM; domain++)
	{
		emimpu_set_domain_secure_access(domain, 1);
	}
}

void emimpu_setup(void) {
	emimpu_set_security_access();
}

static int is_emi_mpu_reg(unsigned int offset)
{
	if(offset >= EMI_MPUA && offset <= EMI_MPUY2)
	{
		return 1;
	}

	return 0;
}


static int is_emi_mpu_reg_write_forbidden(unsigned int offset, unsigned int reg_value)
{
	int i;
	unsigned int no_w_offset[]= {
		EMI_MPUA,
		EMI_MPUI,
		EMI_MPUI_2ND,
		EMI_MPUB,
		EMI_MPUC,
		EMI_MPUJ,
		EMI_MPUJ_2ND,
		EMI_MPUD,
		EMI_MPUE,
		EMI_MPUK,
		EMI_MPUK_2ND,
	};


	if(is_emi_mpu_reg(offset))
	{
		int check_offset = 0;
		if(offset == EMI_MPUK || offset == EMI_MPUK_2ND)
		{
			check_offset = 1;
		}

		if(check_offset)
		{
			unsigned int value = mmio_read_32(EMI_BASE + offset);
			if((reg_value & 0xFFFF) == (value & 0xFFFF))
			{
				return 0;
			}
		}

		for(i = 0; i < sizeof(no_w_offset)/sizeof(unsigned int); i++)
		{
			if(offset == no_w_offset[i])
				return 1;
		}
	}

	return 0;
}


uint64_t sip_emimpu_write(unsigned int offset, unsigned int reg_value)
{
	unsigned int addr;
	if(is_emi_mpu_reg_write_forbidden(offset, reg_value))
		return -3;

	addr = EMI_BASE + offset;
	mmio_write_32(addr, reg_value);
	dsb();
	return 0;
}

uint32_t sip_emimpu_read(unsigned int offset)
{
	unsigned int addr;

	if ((offset >= EMI_CONA) && (offset <= EMI_RFCD))
		addr = EMI_BASE + offset;
	else
		return -3;

	return mmio_read_32(addr);
}
void Debug_log_EMI_MPU(void)
{
	printf("ATF EMI MPUS=0x%x; MPUT=0x%x\n", mmio_read_32(0x102031F0), mmio_read_32(0x102031F8));
}
