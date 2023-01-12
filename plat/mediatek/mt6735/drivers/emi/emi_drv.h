#ifndef _EMI_DRV_H_
#define _EMI_DRV_H_

#define EMI_BASE           0x10203000
#define EMI_CONA           (0x0000)
#define EMI_MPUA           (0x0160)
#define EMI_MPUB           (0x0168)
#define EMI_MPUC           (0x0170)
#define EMI_MPUD           (0x0178)
#define EMI_MPUE           (0x0180)
#define EMI_MPUF	       (0x0188)
#define EMI_MPUG	       (0x0190)
#define EMI_MPUH	       (0x0198)
#define EMI_MPUI	       (0x01A0)
#define EMI_MPUI_2ND	   (0x01A4)
#define EMI_MPUJ           (0x01A8)
#define EMI_MPUJ_2ND	   (0x01AC)
#define EMI_MPUK           (0x01B0)
#define EMI_MPUK_2ND       (0x01B4)
#define EMI_MPUL           (0x01B8)
#define EMI_MPUL_2ND       (0x01BC)
#define EMI_MPUM           (0x01C0)
#define EMI_MPUN           (0x01C8)
#define EMI_MPUO           (0x01D0)
#define EMI_MPUP           (0x01D8)
#define EMI_MPUQ           (0x01E0)
#define EMI_MPUR           (0x01E8)
#define EMI_MPUS           (0x01F0)
#define EMI_MPUT           (0x01F8)
#define EMI_MPUU           (0x0200)
#define EMI_MPUA2		   (0x0260)
#define EMI_MPUB2		   (0x0268)
#define EMI_MPUC2		   (0x0270)
#define EMI_MPUD2		   (0x0278)
#define EMI_MPUE2		   (0x0280)
#define EMI_MPUF2		   (0x0288)
#define EMI_MPUG2		   (0x0290)
#define EMI_MPUH2		   (0x0298)
#define EMI_MPUI2		   (0x02A0)
#define EMI_MPUI2_2ND	   (0x02A4)
#define EMI_MPUJ2		   (0x02A8)
#define EMI_MPUJ2_2ND	   (0x02AC)
#define EMI_MPUK2		   (0x02B0)
#define EMI_MPUK2_2ND	   (0x02B4)
#define EMI_MPUL2		   (0x02B8)
#define EMI_MPUL2_2ND	   (0x02BC)
#define EMI_MPUM2		   (0x02C0)
#define EMI_MPUN2		   (0x02C8)
#define EMI_MPUO2		   (0x02D0)
#define EMI_MPUP2		   (0x02D8)
#define EMI_MPUQ2		   (0x02E0)
#define EMI_MPUR2		   (0x02E8)
#define EMI_MPUU2		   (0x0300)
#define EMI_MPUY2		   (0x0320)
#define EMI_RFCD           (0x0648)

#define EMI_MPU_ALIGNMENT   0x10000
#define EMI_PHY_OFFSET      0x40000000
#define SEC_PHY_SIZE        0x06000000
#define NO_PROTECTION       0
#define SEC_RW              1
#define SEC_RW_NSEC_R       2
#define SEC_RW_NSEC_W       3
#define SEC_R_NSEC_R        4
#define FORBIDDEN           5
#define SECURE_OS_MPU_REGION_ID    0
#define ATF_MPU_REGION_ID          1
#define LOCK                1
#define UNLOCK              0
#define EMIMPU_DX_SEC_BIT   30
#define EMIMPU_DOMAIN_NUM  8
#define EMIMPU_REGION_NUM  16


extern uint64_t sip_emimpu_write(unsigned int offset, unsigned int reg_value);
extern uint32_t sip_emimpu_read(unsigned int offset);
extern void emimpu_set_security_access(void);
extern void emimpu_setup(void);

#endif
