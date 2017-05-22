/*
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the
 *	  distribution.
 *
 *	* Neither the name of Texas Instruments Incorporated nor the names of
 *	  its contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <rsc_types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>
#include <pru_intc.h>
#include <pru_rpmsg.h>
#include "resource_table_1.h"

/* Host-1 Interrupt sets bit 31 in register R31 */
#define HOST_INT ((uint32_t) 1 << 31)

// The PRU-ICSS system events used for RPMsg are defined in the Linux device tree
// PRU1 uses system event 18 (To ARM) and 19 (From ARM)

#define TO_ARM_HOST 18
#define FROM_ARM_HOST 19

#define LSTEP 5 //P8_42
#define LDIR 4 //P8_41
#define RSTEP 2 //P8_43
#define RDIR 3 //P8_44

#define LM0 7 //P8_40
#define LM1 6 //P8_39
#define RM0 0 //P8_45
#define RM1 1 //P8_46

/*
 * Using the name 'rpmsg-client-sample' will probe the RPMsg sample driver
 * found at linux-x.y.z/samples/rpmsg/rpmsg_client_sample.c
 *
 * Using the name 'rpmsg-pru' will probe the rpmsg_pru driver found
 * at linux-x.y.z/drivers/rpmsg/rpmsg_pru.c
 */
#define CHAN_NAME "rpmsg-pru"
#define CHAN_DESC "Channel 31"
#define CHAN_PORT 31

// Used to make sure the Linux drivers are ready for RPMsg communication; Found at linux-x.y.z/include/uapi/linux/virtio_config.h
#define VIRTIO_CONFIG_S_DRIVER_OK 4

volatile register uint32_t __R30;
volatile register uint32_t __R31;

struct DataFrame {
	uint32_t speedLeft;
	uint32_t speedRight;
	uint8_t directionLeft;
	uint8_t directionRight;
	//1 - fullstep; 2 - halfstep; 4 - 1/4 step; 8 - 1/8 step
	uint8_t microstep;
};

uint8_t payload[RPMSG_BUF_SIZE];

void main() {
	struct pru_rpmsg_transport transport;
	struct DataFrame* received;
	received = (struct DataFrame*) malloc(sizeof(struct DataFrame));
	received->speedLeft = 0;
	received->speedRight = 0;
	received->directionLeft = 0;
	received->directionRight = 0;
	received->microstep = 1;
	uint16_t src, dst, len;
	volatile uint8_t *status;
	int32_t stepTargetLeft = 0;
	int32_t stepTargetRight = 0;
	uint32_t speedLeft = 0;
	uint32_t speedRight = 0;

	// Allow OCP master port access by the PRU so the PRU can read external memories
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;
	// Clear the status of the PRU-ICSS system event that the ARM will use to 'kick' us
	CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;

	// Make sure the Linux drivers are ready for RPMsg communication
	status = &resourceTable.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

	// Initialize the RPMsg transport structure
	pru_rpmsg_init(&transport, &resourceTable.rpmsg_vring0, &resourceTable.rpmsg_vring1, TO_ARM_HOST, FROM_ARM_HOST);

	// Create the RPMsg channel between the PRU and ARM user space using the transport structure.
	while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, CHAN_NAME, CHAN_DESC, CHAN_PORT) != PRU_RPMSG_SUCCESS);

	// Enable counter
	PRU1_CTRL.CYCLE = 0;
	PRU1_CTRL.CTRL_bit.CTR_EN = 1;

	while (1) {
		// Check bit 30 of register R31 to see if the ARM has kicked us
		if (__R31 & HOST_INT) {
			// Clear the event status
			CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;
			// Receive all available messages, multiple messages can be sent per kick
			while (pru_rpmsg_receive(&transport, &src, &dst, payload, &len) == PRU_RPMSG_SUCCESS) {
				received = (struct DataFrame*) payload;

				// Set microstepping
				if (received->microstep == 1) {
					// Full step: M0 = 0, M1 = 0
					__R30 = __R30 & ~(1 << LM1);
					__R30 = __R30 & ~(1 << LM0);
					__R30 = __R30 & ~(1 << RM1);
					__R30 = __R30 & ~(1 << RM0);
				} else if (received->microstep == 2) {
					// Half step: M0 = 1, M1 = 0
					__R30 = __R30 & ~(1 << LM1);
					__R30 = __R30 | (1 << LM0);
					__R30 = __R30 & ~(1 << RM1);
					__R30 = __R30 | (1 << RM0);
				} else if (received->microstep == 4) {
					// 1/4 step: M0 = 0, M1 = 1
					__R30 = __R30 | (1 << LM1);
					__R30 = __R30 & ~(1 << LM0);
					__R30 = __R30 | (1 << RM1);
					__R30 = __R30 & ~(1 << RM0);
				} else if (received->microstep == 8) {
					// 1/8 step: M0 = 1, M1 = 1
					__R30 = __R30 | (1 << LM1);
					__R30 = __R30 | (1 << LM0);
					__R30 = __R30 | (1 << RM1);
					__R30 = __R30 | (1 << RM0);
				} else {
					// Invalid microstepping? bail out!
					continue;
				}

				// Set direction of left motor
				if (received->directionLeft == 1) {
					__R30 = __R30 | (1 << LDIR);
				} else {
					__R30 = __R30 & ~(1 << LDIR);
				}
				// Set direction of right motor
				if (received->directionRight == 1) {
					__R30 = __R30 | (1 << RDIR);
				} else {
					__R30 = __R30 & ~(1 << RDIR);
				}

				// Set speeds, check max value (uint->int cast)
				if (received->speedLeft > INT32_MAX) {
					speedLeft = INT32_MAX;
				} else {
					speedLeft = received->speedLeft;
				}
				if (received->speedRight > INT32_MAX) {
					speedRight = INT32_MAX;
				} else {
					speedRight = received->speedRight;
				}
				// stepTargetLeft = speedLeft;
				// stepTargetRight = speedRight;
			}
		}

		// Save current timepoint
		int32_t timeNow = PRU1_CTRL.CYCLE;

		// If enough time has passed, switch step for left motor
		// if (received->speedLeft != 0 && timeNow >= stepTargetLeft) {
		if (timeNow >= stepTargetLeft) {
			// Toggle left motor step
			__R30 = __R30 ^ (1 << LSTEP);

			// Reduce right motor step switch timepoint
			stepTargetRight -= timeNow;
			// Update left motor step switch timepoint
			stepTargetLeft = speedLeft;
			// Reset cycle timer
			PRU1_CTRL.CYCLE = 0;
			timeNow = 0;
		}

		// If enough time has passed, switch step for right motor
		// if (received->speedRight != 0 && timeNow >= stepTargetRight) {
		if (timeNow >= stepTargetRight) {
			// Toggle right motor step
			__R30 = __R30 ^ (1 << RSTEP);

			// Update right motor step switch timepoint
			stepTargetRight += speedRight;
		}
	}
}
