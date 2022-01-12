/*
 * Copyright (c) 2015, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef RTC_H
#define RTC_H

/* RTC registers */
enum {
	RTC_BBPU = 0x4000,
	RTC_IRQ_STA = 0x4002,
	RTC_IRQ_EN = 0x4004,
	RTC_CII_EN = 0x4006
};

enum {
	RTC_OSC32CON = 0x4026,
	RTC_CON = 0x403E,
	RTC_WRTGR = 0x403C
};

enum {
	RTC_PDN1 = 0x402C,
	RTC_PDN2 = 0x402E,
	RTC_SPAR0 = 0x4030,
	RTC_SPAR1 = 0x4032,
	RTC_PROT = 0x4036,
	RTC_DIFF = 0x4038,
	RTC_CALI = 0x403A
};

enum {
	RTC_PROT_UNLOCK1 = 0x586A,
	RTC_PROT_UNLOCK2 = 0x9136
};

enum {
	RTC_BBPU_PWREN	= 1U << 0,
	RTC_BBPU_BBPU	= 1U << 2,
	RTC_BBPU_AUTO	= 1U << 3,
	RTC_BBPU_CLRPKY	= 1U << 4,
	RTC_BBPU_RELOAD	= 1U << 5,
	RTC_BBPU_CBUSY	= 1U << 6
};

enum {
	RTC_BBPU_KEY	= 0x43 << 8
};

/* external API */
uint16_t RTC_Read(uint32_t addr);
void RTC_Write(uint32_t addr, uint16_t data);
int32_t rtc_busy_wait(void);
int32_t RTC_Write_Trigger(void);
int32_t Writeif_unlock(void);
void rtc_bbpu_power_down(void);

#endif /* RTC_H */
