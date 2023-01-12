//SPDX-License-Identifier: BSD-3-Clause

#include <assert.h>
#include <common/debug.h>
#include <lib/mmio.h>
#include <mcucfg.h>
#include <plat_private.h>

#define L2C_SIZE_CFG_OFF    8
#define L2C_SHARE_ENABLE    12

void config_l2_size(void)
{
    unsigned int cache_cfg0;
    
    cache_cfg0 = mmio_read_32((uintptr_t)&mt6735_mcucfg->mp0_ca7l_cache_config) & (0xF << L2C_SIZE_CFG_OFF);
    cache_cfg0 = (cache_cfg0 << 1) | (0x1 << L2C_SIZE_CFG_OFF);
    cache_cfg0 = (mmio_read_32((uintptr_t)&mt6735_mcucfg->mp0_ca7l_cache_config) & ~(0xF << L2C_SIZE_CFG_OFF)) | cache_cfg0;
    mmio_write_32((uintptr_t)&mt6735_mcucfg->mp0_ca7l_cache_config, cache_cfg0);
    cache_cfg0 = mmio_read_32((uintptr_t)&mt6735_mcucfg->mp0_ca7l_cache_config) & ~(0x1 << L2C_SHARE_ENABLE);
    mmio_write_32((uintptr_t)&mt6735_mcucfg->mp0_ca7l_cache_config, cache_cfg0);
}
