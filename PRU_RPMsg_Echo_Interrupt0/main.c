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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>
#include <pru_intc.h>
#include <rsc_types.h>
#include <pru_rpmsg.h>
#include "resource_table_0.h"

struct DistanceFrame {
	uint16_t front;
	uint16_t back;
	uint16_t top;
};

#define TRIG_BIT 4
#define ECHO1_BIT 3
#define ECHO2_BIT 5
#define ECHO3_BIT 2

// Host-0 Interrupt sets bit 30 in register R31
#define HOST_INT ((uint32_t) 1 << 30)

// The PRU-ICSS system events used for RPMsg are defined in the Linux device tree
// PRU0 uses system event 16 (To ARM) and 17 (From ARM)
#define TO_ARM_HOST 16
#define FROM_ARM_HOST 17

// Using the name 'rpmsg-pru' will probe the rpmsg_pru driver found at linux-x.y.z/drivers/rpmsg/rpmsg_pru.c
#define CHAN_NAME "rpmsg-pru"
#define CHAN_DESC "Channel 30"
#define CHAN_PORT 30

#define PRU_OCP_RATE_HZ (200 * 1000 * 1000)

#define TRIG_PULSE_US 10

#define GPIO0_BASE 0x44E07000
#define GPIO1_BASE 0x4804C000
#define GPIO2_BASE 0x481AC000
#define GPIO3_BASE 0x481AE000

#define GPIO2_OE (*(volatile uint32_t *)(GPIO2_BASE + 0x134))
#define GPIO2_DATAIN (*(volatile uint32_t *)(GPIO2_BASE + 0x138))
#define GPIO2_CLEARDATAOUT (*(volatile uint32_t *)(GPIO2_BASE + 0x190))
#define GPIO2_SETDATAOUT (*(volatile uint32_t *)(GPIO2_BASE + 0x194))

// Used to make sure the Linux drivers are ready for RPMsg communication
// Found at linux-x.y.z/include/uapi/linux/virtio_config.h
#define VIRTIO_CONFIG_S_DRIVER_OK	4

volatile register uint32_t __R31;
uint8_t payload[RPMSG_BUF_SIZE];

void sonarsInit() {
	// Enable OCP access
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	// Don't bother with PRU GPIOs. Our timing requirements allow us to use the "slow" system GPIOs.
	// Trigger: output
	GPIO2_OE &= ~(1u << TRIG_BIT);
	// Echos: input
	GPIO2_OE |= (1u << ECHO1_BIT);
	GPIO2_OE |= (1u << ECHO2_BIT);
	GPIO2_OE |= (1u << ECHO3_BIT);
}

// Measure sonars pulse time in microseconds
void sonarsMeasurePulse(uint64_t * pulse1, uint64_t * pulse2, uint64_t * pulse3) {
	// pulse the trigger for 10us
	GPIO2_SETDATAOUT = 1u << TRIG_BIT;
	__delay_cycles(TRIG_PULSE_US * (PRU_OCP_RATE_HZ / 1000000));
	GPIO2_CLEARDATAOUT = 1u << TRIG_BIT;

	// Enable counter
	PRU0_CTRL.CYCLE = 0;
	PRU0_CTRL.CTRL_bit.CTR_EN = 1;

	uint64_t sonar1EchoStart = 0;
	uint64_t sonar2EchoStart = 0;
	uint64_t sonar3EchoStart = 0;
	*pulse1 = 0;
	*pulse2 = 0;
	*pulse3 = 0;

	bool timeout = false;
	// Measure "high" pulse width on all 3 echo pins
	// Until all pulses have been measured or until timeout...
	while (!timeout && (!*pulse1 || !*pulse2 || !*pulse3)) {
		// ... if sonar's echo pulse didn't start...
		if (!sonar1EchoStart) {
			// ... check the pin for being "high"...
			if (GPIO2_DATAIN & (1u << ECHO1_BIT)) {
				// ... and if it is, save sonar's pulse beggining cycle...
				sonar1EchoStart = PRU0_CTRL.CYCLE;
			}
		} else if (!*pulse1) {
			// ... else (if sonar's pulse started), if sonar's pulse didn't end, check pin for being "low"...
			if(!(GPIO2_DATAIN & (1u << ECHO1_BIT))) {
				// ... and if so, save pulse width as difference between "now" and start.
				*pulse1 = PRU0_CTRL.CYCLE - sonar1EchoStart;
			}
		}

		if (!sonar2EchoStart) {
			if (GPIO2_DATAIN & (1u << ECHO2_BIT)) {
				sonar2EchoStart = PRU0_CTRL.CYCLE;
			}
		} else if (!*pulse2) {
			if(!(GPIO2_DATAIN & (1u << ECHO2_BIT))) {
				*pulse2 = PRU0_CTRL.CYCLE - sonar2EchoStart;
			}
		}

		if (!sonar3EchoStart) {
			if (GPIO2_DATAIN & (1u << ECHO3_BIT)) {
				sonar3EchoStart = PRU0_CTRL.CYCLE;
			}
		} else if (!*pulse3) {
			if(!(GPIO2_DATAIN & (1u << ECHO3_BIT))) {
				*pulse3 = PRU0_CTRL.CYCLE - sonar3EchoStart;
			}
		}

		// Also check timeout - 1s.
		timeout = PRU0_CTRL.CYCLE > PRU_OCP_RATE_HZ;
	}

	// Disable counter
	PRU0_CTRL.CTRL_bit.CTR_EN = 0;

	if (timeout) {
		*pulse1 = -1;
		*pulse2 = -1;
		*pulse3 = -1;
		return;
	}

	*pulse1 /= ((uint64_t)PRU_OCP_RATE_HZ / 1000000);
	*pulse2 /= ((uint64_t)PRU_OCP_RATE_HZ / 1000000);
	*pulse3 /= ((uint64_t)PRU_OCP_RATE_HZ / 1000000);
}

void sonarsMeasureDistance(uint16_t * distance1, uint16_t * distance2, uint16_t * distance3) {
	uint64_t pulse1, pulse2, pulse3;
	sonarsMeasurePulse(&pulse1, &pulse2, &pulse3);

	*distance1 = pulse1 * 1000 / 5844;
	*distance2 = pulse2 * 1000 / 5844;
	*distance3 = pulse3 * 1000 / 5844;
}

void main() {
	struct pru_rpmsg_transport transport;
	uint16_t src, dst, len;
	volatile uint8_t *status;
	struct DistanceFrame* distances;
	distances = (struct DistanceFrame*) malloc(sizeof(struct DistanceFrame));

	sonarsInit();

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

	uint16_t distanceFront, distanceBack, distanceTop;
	while (1) {
		// Check bit 30 of register R31 to see if the ARM has kicked us
		if (!(__R31 & HOST_INT)) {
			continue;
		}

		// Clear the event status
		CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;
		// Receive all available messages - multiple messages can be sent per kick
		while (pru_rpmsg_receive(&transport, &src, &dst, payload, &len) == PRU_RPMSG_SUCCESS) {
			// Do the measurement
			sonarsMeasureDistance(&distanceFront, &distanceBack, &distanceTop);
			distances->front = distanceFront;
			distances->back = distanceBack;
			distances->top = distanceTop;
			// Echo the message back to the same address from which we just received
			pru_rpmsg_send(&transport, dst, src, distances, len);
		}
	}
}
