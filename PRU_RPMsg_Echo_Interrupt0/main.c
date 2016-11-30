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

volatile register uint32_t __R31;

struct distance_frame{
	uint16_t front;
	uint16_t back;
	uint16_t top;
};

/* Host-0 Interrupt sets bit 30 in register R31 */
#define HOST_INT			((uint32_t) 1 << 30)

/* The PRU-ICSS system events used for RPMsg are defined in the Linux device tree
 * PRU0 uses system event 16 (To ARM) and 17 (From ARM)
 * PRU1 uses system event 18 (To ARM) and 19 (From ARM)
 */
#define TO_ARM_HOST			16
#define FROM_ARM_HOST			17

/*
 * Using the name 'rpmsg-pru' will probe the rpmsg_pru driver found
 * at linux-x.y.z/drivers/rpmsg/rpmsg_pru.c
 */
#define CHAN_NAME			"rpmsg-pru"
#define CHAN_DESC			"Channel 30"
#define CHAN_PORT			30


#define PRU_OCP_RATE_HZ		(200 * 1000 * 1000)

#define TRIG_PULSE_US		10

#define GPIO0_BASE 0x44E07000
#define GPIO1_BASE 0x4804C000
#define GPIO2_BASE 0x481AC000
#define GPIO3_BASE 0x481AE000

#define GPIO2_OE		(*(volatile uint32_t *)(GPIO2_BASE + 0x134))
#define GPIO2_DATAIN		(*(volatile uint32_t *)(GPIO2_BASE + 0x138))
#define GPIO2_CLEARDATAOUT	(*(volatile uint32_t *)(GPIO2_BASE + 0x190))
#define GPIO2_SETDATAOUT	(*(volatile uint32_t *)(GPIO2_BASE + 0x194))

#define TRIG_BIT		4 //to change used by wlink
#define ECHO1_BIT		3 //to change used by wlink
#define ECHO2_BIT		5 //to change used by wlink
#define ECHO3_BIT		2 //to change used by wlink


/*
 * Used to make sure the Linux drivers are ready for RPMsg communication
 * Found at linux-x.y.z/include/uapi/linux/virtio_config.h
 */
#define VIRTIO_CONFIG_S_DRIVER_OK	4

uint8_t payload[RPMSG_BUF_SIZE];

void hc_sr04_init(void)
{
	/* Enable OCP access */
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	/*
	 * Don't bother with PRU GPIOs. Our timing requirements allow
	 * us to use the "slow" system GPIOs.
	 */
	GPIO2_OE &= ~(1u << TRIG_BIT);	/* output */
	GPIO2_OE |= (1u << ECHO1_BIT);	/* input */
	GPIO2_OE |= (1u << ECHO2_BIT);	/* input */
	GPIO2_OE |= (1u << ECHO3_BIT);	/* input */
}

int hc_sr04_measure_pulse(int snumber) //snumber - number of sonar from 1 to 3
{
	bool echo, timeout;

	/* pulse the trigger for 10us */
	GPIO2_SETDATAOUT = 1u << TRIG_BIT;
	__delay_cycles(TRIG_PULSE_US * (PRU_OCP_RATE_HZ / 1000000));
	GPIO2_CLEARDATAOUT = 1u << TRIG_BIT;

	/* Enable counter */
	PRU0_CTRL.CYCLE = 0;
	PRU0_CTRL.CTRL_bit.CTR_EN = 1;

	/* wait for ECHO to get high */
	do {
		if(snumber ==1)echo = !!(GPIO2_DATAIN & (1u << ECHO1_BIT));
		if(snumber ==2)echo = !!(GPIO2_DATAIN & (1u << ECHO2_BIT));
		if(snumber ==3)echo = !!(GPIO2_DATAIN & (1u << ECHO3_BIT));
		timeout = PRU0_CTRL.CYCLE > PRU_OCP_RATE_HZ;
	} while (!echo && !timeout);

	PRU0_CTRL.CTRL_bit.CTR_EN = 0;

	if (timeout)
		return -1;

	/* Restart the counter */
	PRU0_CTRL.CYCLE = 0;
	PRU0_CTRL.CTRL_bit.CTR_EN = 1;

	/* measure the "high" pulse length */
	do {
		if(snumber ==1)echo = !!(GPIO2_DATAIN & (1u << ECHO1_BIT));
		if(snumber ==2)echo = !!(GPIO2_DATAIN & (1u << ECHO2_BIT));
		if(snumber ==3)echo = !!(GPIO2_DATAIN & (1u << ECHO3_BIT));
		timeout = PRU0_CTRL.CYCLE > PRU_OCP_RATE_HZ;
	} while (echo && !timeout);

	PRU0_CTRL.CTRL_bit.CTR_EN = 0;

	if (timeout)
		return -1;

	uint64_t cycles = PRU0_CTRL.CYCLE;

	return cycles / ((uint64_t)PRU_OCP_RATE_HZ / 1000000);
}


uint16_t measure_distance_mm(int snumber)//snumber - sonar number from 1 to 3
{
	int t_us = hc_sr04_measure_pulse(snumber);
	int d_mm;

	/*
	 * Print the distance received from the sonar
	 * At 20 degrees in dry air the speed of sound is 3422 mm/sec
	 * so it takes 2.912 us to make 1 mm, i.e. 5.844 us for a
	 * roundtrip of 1 mm.
	 */
	d_mm = (t_us * 1000) / 5844;
	if (t_us < 0)
		d_mm = -1;

	return d_mm;
}


/*
 * main.c
 */
void main(void)
{
	struct pru_rpmsg_transport transport;
	uint16_t src, dst, len;
	volatile uint8_t *status;
	struct distance_frame* distances;
	distances = (struct distance_frame*) malloc(sizeof(struct distance_frame));


	hc_sr04_init(); //sonars initialization
	/* Allow OCP master port access by the PRU so the PRU can read external memories */
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	/* Clear the status of the PRU-ICSS system event that the ARM will use to 'kick' us */
	CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;

	/* Make sure the Linux drivers are ready for RPMsg communication */
	status = &resourceTable.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

	/* Initialize the RPMsg transport structure */
	pru_rpmsg_init(&transport, &resourceTable.rpmsg_vring0, &resourceTable.rpmsg_vring1, TO_ARM_HOST, FROM_ARM_HOST);
	/* Create the RPMsg channel between the PRU and ARM user space using the transport structure. */
	while (pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, CHAN_NAME, CHAN_DESC, CHAN_PORT) != PRU_RPMSG_SUCCESS);
	while (1) {
		/* Check bit 30 of register R31 to see if the ARM has kicked us */
		if (__R31 & HOST_INT) {
			/* Clear the event status */
			CT_INTC.SICR_bit.STS_CLR_IDX = FROM_ARM_HOST;
			/* Receive all available messages, multiple messages can be sent per kick */
			while (pru_rpmsg_receive(&transport, &src, &dst, payload, &len) == PRU_RPMSG_SUCCESS) {
				/* Echo the message back to the same address from which we just received */
				distances->front=(uint16_t)measure_distance_mm(1);
				distances->back=(uint16_t)measure_distance_mm(2);
				distances->top=(uint16_t)measure_distance_mm(3);

				pru_rpmsg_send(&transport, dst, src, distances, len);
			}
		}
	}
}
