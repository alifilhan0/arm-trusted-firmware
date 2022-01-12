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

#ifndef __PSCI_H__
#define __PSCI_H__


/*******************************************************************************
 * Defines for runtime services func ids
 ******************************************************************************/

/*
 * Number of PSCI calls (above) implemented
 */
/*******************************************************************************
 * PSCI Migrate and friends
 ******************************************************************************/
#define PSCI_TOS_UP_MIG_CAP	0
#define PSCI_TOS_NOT_UP_MIG_CAP	1
#define PSCI_TOS_NOT_PRESENT_MP	2

/*******************************************************************************
 * PSCI CPU_SUSPEND 'power_state' parameter specific defines
 ******************************************************************************/

#define psci_get_pstate_id(pstate)	(pstate >> PSTATE_ID_SHIFT) & \
					PSTATE_ID_MASK
#define psci_get_pstate_type(pstate)	(pstate >> PSTATE_TYPE_SHIFT) & \
					PSTATE_TYPE_MASK
#define psci_get_pstate_afflvl(pstate)	(pstate >> PSTATE_AFF_LVL_SHIFT) & \
					PSTATE_AFF_LVL_MASK

/*******************************************************************************
 * PSCI version
 ******************************************************************************/

/*******************************************************************************
 * PSCI error codes
 ******************************************************************************/
#define PSCI_E_SUCCESS		0
#define PSCI_E_NOT_SUPPORTED	-1
#define PSCI_E_INVALID_PARAMS	-2
#define PSCI_E_DENIED		-3
#define PSCI_E_ALREADY_ON	-4
#define PSCI_E_ON_PENDING	-5
#define PSCI_E_INTERN_FAIL	-6
#define PSCI_E_NOT_PRESENT	-7
#define PSCI_E_DISABLED		-8

/*******************************************************************************
 * PSCI affinity state related constants. An affinity instance could be present
 * or absent physically to cater for asymmetric topologies. If present then it
 * could in one of the 4 further defined states.
 ******************************************************************************/
#define PSCI_STATE_SHIFT	1
#define PSCI_STATE_MASK		0xff

#define PSCI_AFF_ABSENT		0x0
#define PSCI_AFF_PRESENT	0x1
#define PSCI_STATE_ON		0x0
#define PSCI_STATE_OFF		0x1
#define PSCI_STATE_ON_PENDING	0x2
#define PSCI_STATE_SUSPEND	0x3

#define PSCI_INVALID_DATA -1

#define get_phys_state(x)	(x != PSCI_STATE_ON ? \
				 PSCI_STATE_OFF : PSCI_STATE_ON)

#define psci_validate_power_state(pstate) (pstate & PSTATE_VALID_MASK)


#ifndef __ASSEMBLY__

#include <stdint.h>

/*******************************************************************************
 * Structure used to store per-cpu information relevant to the PSCI service.
 * It is populated in the per-cpu data array. In return we get a guarantee that
 * this information will not reside on a cache line shared with another cpu.
 ******************************************************************************/
/*******************************************************************************
 * Structure populated by platform specific code to export routines which
 * perform common low level pm functions
 ******************************************************************************/
/*******************************************************************************
 * Optional structure populated by the Secure Payload Dispatcher to be given a
 * chance to perform any bookkeeping before PSCI executes a power mgmt.
 * operation. It also allows PSCI to determine certain properties of the SP e.g.
 * migrate capability etc.
 ******************************************************************************/

/*******************************************************************************
 * Function & Data prototypes
 ******************************************************************************/


#endif /*__ASSEMBLY__*/


#endif /* __PSCI_H__ */
