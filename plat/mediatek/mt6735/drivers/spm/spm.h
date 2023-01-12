/*
 * Copyright (c) 2019, MediaTek Inc. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SPM_H
#define SPM_H

#define SPM_POWERON_CONFIG_SET          0x000
#define SPM_POWER_ON_VAL0               0x010
#define SPM_POWER_ON_VAL1               0x014
#define SPM_CLK_SETTLE                  0x100
#define SPM_CA7_CPU0_PWR_CON            0x200
#define SPM_CA7_DBG_PWR_CON             0x204
#define SPM_CA7_CPUTOP_PWR_CON          0x208
#define SPM_VDE_PWR_CON                 0x210
#define SPM_MFG_PWR_CON                 0x214
#define SPM_CA7_CPU1_PWR_CON            0x218
#define SPM_CA7_CPU2_PWR_CON            0x21c
#define SPM_CA7_CPU3_PWR_CON            0x220
#define SPM_VEN_PWR_CON                 0x230
#define SPM_IFR_PWR_CON                 0x234
#define SPM_ISP_PWR_CON                 0x238
#define SPM_DIS_PWR_CON                 0x23c
#define SPM_DPY_PWR_CON                 0x240
#define SPM_CA7_CPUTOP_L2_PDN           0x244
#define SPM_CA7_CPUTOP_L2_SLEEP         0x248
#define SPM_CA7_CPU0_L1_PDN             0x25c
#define SPM_CA7_CPU1_L1_PDN             0x264
#define SPM_CA7_CPU2_L1_PDN             0x26c
#define SPM_CA7_CPU3_L1_PDN             0x274
#define SPM_GCPU_SRAM_CON               0x27c
#define SPM_CONN_PWR_CON                0x280
#define SPM_MD_PWR_CON                  0x284
#define SPM_MCU_PWR_CON                 0x290
#define SPM_IFR_SRAMROM_CON             0x294
#define SPM_MJC_PWR_CON                 0x298
#define SPM_AUDIO_PWR_CON               0x29c
#define SPM_CA15_CPU0_PWR_CON           0x2a0
#define SPM_CA15_CPU1_PWR_CON           0x2a4
#define SPM_CA15_CPU2_PWR_CON           0x2a8
#define SPM_CA15_CPU3_PWR_CON           0x2ac
#define SPM_CA15_CPUTOP_PWR_CON         0x2b0
#define SPM_CA15_L1_PWR_CON             0x2b4
#define SPM_CA15_L2_PWR_CON             0x2b8
#define SPM_MFG_2D_PWR_CON              0x2c0
#define SPM_MFG_ASYNC_PWR_CON           0x2c4
#define SPM_MD32_SRAM_CON               0x2c8
#define SPM_ARMPLL_DIV_PWR_CON          0x2cc
#define SPM_MD2_PWR_CON                 0x2d0
#define SPM_C2K_PWR_CON                 0x2d4
#define SPM_INFRA_MD_PWR_CON            0x2d8
#define SPM_CPU_EXT_ISO                 0x2dc
#define SPM_PCM_CON0                    0x310
#define SPM_PCM_CON1                    0x314
#define SPM_PCM_IM_PTR                  0x318
#define SPM_PCM_IM_LEN                  0x31c
#define SPM_PCM_REG_DATA_INI            0x320
#define SPM_PCM_EVENT_VECTOR0           0x340
#define SPM_PCM_EVENT_VECTOR1           0x344
#define SPM_PCM_EVENT_VECTOR2           0x348
#define SPM_PCM_EVENT_VECTOR3           0x34c
#define SPM_PCM_MAS_PAUSE_MASK          0x354
#define SPM_PCM_PWR_IO_EN               0x358
#define SPM_PCM_TIMER_VAL               0x35c
#define SPM_PCM_TIMER_OUT               0x360
#define SPM_PCM_REG0_DATA               0x380
#define SPM_PCM_REG1_DATA               0x384
#define SPM_PCM_REG2_DATA               0x388
#define SPM_PCM_REG3_DATA               0x38c
#define SPM_PCM_REG4_DATA               0x390
#define SPM_PCM_REG5_DATA               0x394
#define SPM_PCM_REG6_DATA               0x398
#define SPM_PCM_REG7_DATA               0x39c
#define SPM_PCM_REG8_DATA               0x3a0
#define SPM_PCM_REG9_DATA               0x3a4
#define SPM_PCM_REG10_DATA              0x3a8
#define SPM_PCM_REG11_DATA              0x3ac
#define SPM_PCM_REG12_DATA              0x3b0
#define SPM_PCM_REG13_DATA              0x3b4
#define SPM_PCM_REG14_DATA              0x3b8
#define SPM_PCM_REG15_DATA              0x3bc
#define SPM_PCM_EVENT_REG_STA           0x3c0
#define SPM_PCM_FSM_STA                 0x3c4
#define SPM_PCM_IM_HOST_RW_PTR          0x3c8
#define SPM_PCM_IM_HOST_RW_DAT          0x3cc
#define SPM_PCM_EVENT_VECTOR4           0x3d0
#define SPM_PCM_EVENT_VECTOR5           0x3d4
#define SPM_PCM_EVENT_VECTOR6           0x3d8
#define SPM_PCM_EVENT_VECTOR7           0x3dc
#define SPM_PCM_SW_INT_SET              0x3e0
#define SPM_PCM_SW_INT_CLEAR            0x3e4
#define SPM_CLK_CON                     0x400
#define SPM_SLEEP_DUAL_VCORE_PWR_CON    0x404
#define SPM_SLEEP_PTPOD2_CON            0x408
#define SPM_APMCU_PWRCTL                0x600
#define SPM_AP_DVFS_CON_SET             0x604
#define SPM_AP_STANBY_CON               0x608
#define SPM_PWR_STATUS                  0x60c
#define SPM_PWR_STATUS_2ND              0x610
#define SPM_SLEEP_MDBSI_CON             0x614
#define SPM_BSI_GEN                     0x620
#define SPM_BSI_EN_SR                   0x624
#define SPM_BSI_CLK_SR                  0x628
#define SPM_BSI_DO_SR                   0x62c
#define SPM_BSI_D1_SR                   0x630
#define SPM_BSI_D2_SR                   0x634
#define SPM_AP_SEMA                     0x638
#define SPM_SPM_SEMA                    0x63c
#define SPM_SLEEP_TIMER_STA             0x720
#define SPM_SLEEP_TWAM_CON              0x760
#define SPM_SLEEP_TWAM_STATUS0          0x764
#define SPM_SLEEP_TWAM_STATUS1          0x768
#define SPM_SLEEP_TWAM_STATUS2          0x76c
#define SPM_SLEEP_TWAM_STATUS3          0x770
#define SPM_SLEEP_TWAM_CURR_STATUS0     0x774
#define SPM_SLEEP_TWAM_CURR_STATUS1     0x778
#define SPM_SLEEP_TWAM_CURR_STATUS2     0x77c
#define SPM_SLEEP_TWAM_CURR_STATUS3     0x780
#define SPM_SLEEP_TWAM_TIMER_OUT        0x784
#define SPM_SLEEP_TWAM_WINDOW_LEN       0x788
#define SPM_SLEEP_IDLE_SEL              0x78C
#define SPM_SLEEP_WAKEUP_EVENT_MASK     0x810
#define SPM_SLEEP_CPU_WAKEUP_EVENT      0x814
#define SPM_SLEEP_MD32_WAKEUP_EVENT_MASK    0x818
#define SPM_PCM_WDT_TIMER_VAL           0x824
#define SPM_PCM_WDT_TIMER_OUT           0x828
#define SPM_PCM_MD32_MAILBOX            0x830
#define SPM_PCM_MD32_IRQ                0x834
#define SPM_SLEEP_ISR_MASK              0x900
#define SPM_SLEEP_ISR_STATUS            0x904
#define SPM_SLEEP_ISR_RAW_STA           0x910
#define SPM_SLEEP_MD32_ISR_RAW_STA      0x914
#define SPM_SLEEP_WAKEUP_MISC           0x918
#define SPM_SLEEP_BUS_PROTECT_RDY       0x91c
#define SPM_SLEEP_SUBSYS_IDLE_STA       0x920
#define SPM_PCM_RESERVE                 0xb00
#define SPM_PCM_RESERVE2                0xb04
#define SPM_PCM_FLAGS                   0xb08
#define SPM_PCM_SRC_REQ                 0xb0c
#define SPM_PCM_DEBUG_CON               0xb20
#define SPM_CA7_CPU0_IRQ_MASK           0xb30
#define SPM_CA7_CPU1_IRQ_MASK           0xb34
#define SPM_CA7_CPU2_IRQ_MASK           0xb38
#define SPM_CA7_CPU3_IRQ_MASK           0xb3c
#define SPM_CA15_CPU0_IRQ_MASK          0xb40
#define SPM_CA15_CPU1_IRQ_MASK          0xb44
#define SPM_CA15_CPU2_IRQ_MASK          0xb48
#define SPM_CA15_CPU3_IRQ_MASK          0xb4c
#define SPM_PCM_PASR_DPD_0              0xb60
#define SPM_PCM_PASR_DPD_1              0xb64
#define SPM_PCM_PASR_DPD_2              0xb68
#define SPM_PCM_PASR_DPD_3              0xb6c
#define SPM_SLEEP_CA7_WFI0_EN           0xf00
#define SPM_SLEEP_CA7_WFI1_EN           0xf04
#define SPM_SLEEP_CA7_WFI2_EN           0xf08
#define SPM_SLEEP_CA7_WFI3_EN           0xf0c
#define SPM_SLEEP_CA15_WFI0_EN          0xf10
#define SPM_SLEEP_CA15_WFI1_EN          0xf14
#define SPM_SLEEP_CA15_WFI2_EN          0xf18
#define SPM_SLEEP_CA15_WFI3_EN          0xf1c

#define SPM_PROJECT_CODE		0xb16
#define SPM_REGWR_EN			BIT(0)
#define SPM_REGWR_CFG_KEY		(SPM_PROJECT_CODE << 16)

#define AP_PLL_CON3			0x1020900c
#define AP_PLL_CON4			0x10209010

#define PWR_RST_BIT			BIT(0)
#define PWR_ISO_BIT			BIT(1)
#define PWR_ON_BIT			BIT(2)
#define PWR_ON_2ND_BIT			BIT(3)
#define PWR_CLK_DIS_BIT			BIT(4)
#define SRAM_CKISO_BIT			BIT(5)
#define SRAM_ISOINT_BIT			BIT(6)

#define L1_PDN				BIT(0)
#define L1_PDN_ACK			BIT(8)
#define	MP0_CPUTOP			BIT(9)
#define	MP0_CPU3			BIT(10)
#define	MP0_CPU2			BIT(11)
#define	MP0_CPU1			BIT(12)
#define	MP0_CPU0			BIT(13)

/* SPM_SLEEP_TIMER_STA		0x720) */
#define	MP0_CPU0_STANDBYWFI		BIT(16)
#define	MP0_CPU1_STANDBYWFI		BIT(17)
#define	MP0_CPU2_STANDBYWFI		BIT(18)
#define	MP0_CPU3_STANDBYWFI		BIT(19)
#define	MP0_CPUTOP_STANDBYWFI		BIT(24)

#define SPM_CPU_PDN_DIS			BIT(0)
#define SPM_INFRA_PDN_DIS		BIT(1)
#define SPM_DDRPHY_PDN_DIS		BIT(2)
#define SPM_DUALVCORE_PDN_DIS		BIT(3)
#define SPM_PASR_DIS			BIT(4)
#define SPM_DPD_DIS			BIT(5)
#define SPM_SODI_DIS			BIT(6)
#define SPM_MEMPLL_RESET		BIT(7)
#define SPM_MAINPLL_PDN_DIS		BIT(8)
#define SPM_CPU_DVS_DIS			BIT(9)
#define SPM_CPU_DORMANT			BIT(10)
#define SPM_EXT_VSEL_GPIO103		BIT(11)
#define SPM_DDR_HIGH_SPEED		BIT(12)
#define SPM_OPT				BIT(13)

#define POWER_ON_VAL1_DEF		0x01011820
#define PCM_FSM_STA_DEF			0x48490
#define PCM_END_FSM_STA_DEF		0x08490
#define PCM_END_FSM_STA_MASK		0x3fff0
#define PCM_HANDSHAKE_SEND1		0xbeefbeef

#define PCM_WDT_TIMEOUT			(30 * 32768)
#define PCM_TIMER_MAX			(0xffffffff - PCM_WDT_TIMEOUT)

#define CON0_PCM_KICK			BIT(0)
#define CON0_IM_KICK			BIT(1)
#define CON0_IM_SLEEP_DVS		BIT(3)
#define CON0_PCM_SW_RESET		BIT(15)
#define CON0_CFG_KEY			(SPM_PROJECT_CODE << 16)

#define CON1_IM_SLAVE			BIT(0)
#define CON1_MIF_APBEN			BIT(3)
#define CON1_PCM_TIMER_EN		BIT(5)
#define CON1_IM_NONRP_EN		BIT(6)
#define CON1_PCM_WDT_EN			BIT(8)
#define CON1_PCM_WDT_WAKE_MODE		BIT(9)
#define CON1_SPM_SRAM_SLP_B		BIT(10)
#define CON1_SPM_SRAM_ISO_B		BIT(11)
#define CON1_EVENT_LOCK_EN		BIT(12)
#define CON1_CFG_KEY			(SPM_PROJECT_CODE << 16)

#define PCM_PWRIO_EN_R0			BIT(0)
#define PCM_PWRIO_EN_R7			BIT(7)
#define PCM_RF_SYNC_R0			BIT(16)
#define PCM_RF_SYNC_R2			BIT(18)
#define PCM_RF_SYNC_R6			BIT(22)
#define PCM_RF_SYNC_R7			BIT(23)

#define CC_SYSCLK0_EN_0			BIT(0)
#define CC_SYSCLK0_EN_1			BIT(1)
#define CC_SYSCLK1_EN_0			BIT(2)
#define CC_SYSCLK1_EN_1			BIT(3)
#define CC_SYSSETTLE_SEL		BIT(4)
#define CC_LOCK_INFRA_DCM		BIT(5)
#define CC_SRCLKENA_MASK_0		BIT(6)
#define CC_CXO32K_RM_EN_MD1		BIT(9)
#define CC_CXO32K_RM_EN_MD2		BIT(10)
#define CC_CLKSQ1_SEL			BIT(12)
#define CC_DISABLE_DORM_PWR		BIT(14)
#define CC_MD32_DCM_EN			BIT(18)

#define WFI_OP_AND			1
#define WFI_OP_OR			0

#define WAKE_MISC_PCM_TIMER		BIT(19)
#define WAKE_MISC_CPU_WAKE		BIT(20)

/* define WAKE_SRC_XXX */
#define WAKE_SRC_SPM_MERGE		BIT(0)
#define WAKE_SRC_KP			BIT(2)
#define WAKE_SRC_WDT			BIT(3)
#define WAKE_SRC_GPT			BIT(4)
#define WAKE_SRC_EINT			BIT(6)
#define WAKE_SRC_LOW_BAT		BIT(9)
#define WAKE_SRC_MD32			BIT(10)
#define WAKE_SRC_USB_CD			BIT(14)
#define WAKE_SRC_USB_PDN		BIT(15)
#define WAKE_SRC_AFE			BIT(20)
#define WAKE_SRC_THERM			BIT(21)
#define WAKE_SRC_CIRQ			BIT(22)
#define WAKE_SRC_SYSPWREQ		BIT(24)
#define WAKE_SRC_SEJ			BIT(27)
#define WAKE_SRC_ALL_MD32		BIT(28)
#define WAKE_SRC_CPU_IRQ		BIT(29)

enum wake_reason_t {
	WR_NONE = 0,
	WR_UART_BUSY = 1,
	WR_PCM_ASSERT = 2,
	WR_PCM_TIMER = 3,
	WR_PCM_ABORT = 4,
	WR_WAKE_SRC = 5,
	WR_UNKNOWN = 6,
};

struct pwr_ctrl {
	unsigned int pcm_flags;
	unsigned int pcm_flags_cust;
	unsigned int pcm_reserve;
	unsigned int timer_val;
	unsigned int timer_val_cust;
	unsigned int wake_src;
	unsigned int wake_src_cust;
	unsigned int wake_src_md32;
	unsigned short r0_ctrl_en;
	unsigned short r7_ctrl_en;
	unsigned short infra_dcm_lock;
	unsigned short pcm_apsrc_req;
	unsigned short mcusys_idle_mask;
	unsigned short ca15top_idle_mask;
	unsigned short ca7top_idle_mask;
	unsigned short wfi_op;
	unsigned short ca15_wfi0_en;
	unsigned short ca15_wfi1_en;
	unsigned short ca15_wfi2_en;
	unsigned short ca15_wfi3_en;
	unsigned short ca7_wfi0_en;
	unsigned short ca7_wfi1_en;
	unsigned short ca7_wfi2_en;
	unsigned short ca7_wfi3_en;
	unsigned short disp_req_mask;
	unsigned short mfg_req_mask;
	unsigned short md32_req_mask;
	unsigned short syspwreq_mask;
	unsigned short srclkenai_mask;
};

struct wake_status {
	unsigned int assert_pc;
	unsigned int r12;
	unsigned int raw_sta;
	unsigned int wake_misc;
	unsigned int timer_out;
	unsigned int r13;
	unsigned int idle_sta;
	unsigned int debug_flag;
	unsigned int event_reg;
	unsigned int isr;
};

struct pcm_desc {
	const char *version;		/* PCM code version */
	const unsigned int *base;	/* binary array base */
	const unsigned int size;	/* binary array size */
	const unsigned char sess;	/* session number */
	const unsigned char replace;	/* replace mode */

	unsigned int vec0;		/* event vector 0 config */
	unsigned int vec1;		/* event vector 1 config */
	unsigned int vec2;		/* event vector 2 config */
	unsigned int vec3;		/* event vector 3 config */
	unsigned int vec4;		/* event vector 4 config */
	unsigned int vec5;		/* event vector 5 config */
	unsigned int vec6;		/* event vector 6 config */
	unsigned int vec7;		/* event vector 7 config */
};

struct spm_lp_scen {
	const struct pcm_desc *pcmdesc;
	struct pwr_ctrl *pwrctrl;
};

#define EVENT_VEC(event, resume, imme, pc)	\
	(((pc) << 16) |				\
	 (!!(imme) << 6) |			\
	 (!!(resume) << 5) |			\
	 ((event) & 0x1f))

#define spm_read(addr)		mmio_read_32(addr)
#define spm_write(addr, val)	mmio_write_32(addr, val)

#define is_cpu_pdn(flags)	(!((flags) & SPM_CPU_PDN_DIS))
#define is_infra_pdn(flags)	(!((flags) & SPM_INFRA_PDN_DIS))
#define is_ddrphy_pdn(flags)	(!((flags) & SPM_DDRPHY_PDN_DIS))

static inline void set_pwrctrl_pcm_flags(struct pwr_ctrl *pwrctrl,
					 unsigned int flags)
{
	flags &= ~SPM_EXT_VSEL_GPIO103;

	if (pwrctrl->pcm_flags_cust == 0)
		pwrctrl->pcm_flags = flags;
	else
		pwrctrl->pcm_flags = pwrctrl->pcm_flags_cust;
}

static inline void set_pwrctrl_pcm_data(struct pwr_ctrl *pwrctrl,
					unsigned int data)
{
	pwrctrl->pcm_reserve = data;
}

void spm_reset_and_init_pcm(void);

void spm_init_pcm_register(void);	/* init r0 and r7 */
void spm_set_power_control(const struct pwr_ctrl *pwrctrl);
void spm_set_wakeup_event(const struct pwr_ctrl *pwrctrl);

void spm_get_wakeup_status(struct wake_status *wakesta);
void spm_set_sysclk_settle(void);
void spm_kick_pcm_to_run(struct pwr_ctrl *pwrctrl);
void spm_clean_after_wakeup(void);
enum wake_reason_t spm_output_wake_reason(struct wake_status *wakesta);
void spm_register_init(void);
void spm_go_to_hotplug(void);
void spm_init_event_vector(const struct pcm_desc *pcmdesc);
void spm_kick_im_to_fetch(const struct pcm_desc *pcmdesc);
int is_mcdi_ready(void);
int is_hotplug_ready(void);
int is_suspend_ready(void);
void set_mcdi_ready(void);
void set_hotplug_ready(void);
void set_suspend_ready(void);
void clear_all_ready(void);
void spm_lock_init(void);
void spm_lock_get(void);
void spm_lock_release(void);
void spm_boot_init(void);

#endif /* SPM_H */
