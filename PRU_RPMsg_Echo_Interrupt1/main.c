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
#include <pru_cfg.h>
#include <pru_intc.h>
#include <rsc_types.h>
#include <pru_rpmsg.h>
#include "resource_table_1.h"

volatile register uint32_t __R30;
volatile register uint32_t __R31;


/* Host-1 Interrupt sets bit 31 in register R31 */
#define HOST_INT			((uint32_t) 1 << 31)	

/* The PRU-ICSS system events used for RPMsg are defined in the Linux device tree
 * PRU0 uses system event 16 (To ARM) and 17 (From ARM)
 * PRU1 uses system event 18 (To ARM) and 19 (From ARM)
 */
#define TO_ARM_HOST			18	
#define FROM_ARM_HOST			19

#define LSTEP				5 //P8_42
#define LDIR				4 //P8_41
#define RSTEP				2 //P8_43
#define RDIR				3 //P8_44

#define LM0					7 //P8_40
#define LM1					6 //P8_39
#define RM0					0 //P8_45
#define RM1					1 //P8_46

/*
 * Using the name 'rpmsg-client-sample' will probe the RPMsg sample driver
 * found at linux-x.y.z/samples/rpmsg/rpmsg_client_sample.c
 *
 * Using the name 'rpmsg-pru' will probe the rpmsg_pru driver found
 * at linux-x.y.z/drivers/rpmsg/rpmsg_pru.c
 */
//#define CHAN_NAME			"rpmsg-client-sample"
#define CHAN_NAME			"rpmsg-pru"

#define CHAN_DESC			"Channel 31"
#define CHAN_PORT			31

struct data_frame{
	unsigned int speedl;
	unsigned int speedr;
	uint8_t dirl;
	uint8_t dirr;
	uint8_t mstep; //1 - fullstep 2 - halfstep 3 - 1/4step 4 1/8 step
};

/*
 * Used to make sure the Linux drivers are ready for RPMsg communication
 * Found at linux-x.y.z/include/uapi/linux/virtio_config.h
 */
#define VIRTIO_CONFIG_S_DRIVER_OK	4

uint8_t payload[RPMSG_BUF_SIZE];

/*
 * main.c
 */
void main(void)
{
	struct pru_rpmsg_transport transport;
	struct data_frame* received;
	received = (struct data_frame*) malloc(sizeof(struct data_frame));
	received->speedl = 0;
	received->speedr = 0;
	uint16_t src, dst, len;
	volatile uint8_t *status;
	unsigned int i;
	unsigned int j=0;
	unsigned int k=0;


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
				//pru_rpmsg_send(&transport, dst, src, payload, len);
				received = (struct data_frame*)payload;
				if(received->dirl==1){
					__R30 = __R30 | (1 << LDIR);
				}
				else{
					__R30 = __R30 & ~(1 << LDIR);
				}
				if(received->dirr==1){
					__R30 = __R30 | (1 << RDIR);
				}
				else{
					__R30 = __R30 & ~(1 << RDIR);
				}
				if(received->mstep==1){
					__R30 = __R30 & ~(1 << LM1);
					__R30 = __R30 & ~(1 << LM0);
					__R30 = __R30 & ~(1 << RM1);
					__R30 = __R30 & ~(1 << RM0);
				}
				if(received->mstep==2){
					__R30 = __R30 & ~(1 << LM1);
					__R30 = __R30 | (1 << LM0);
					__R30 = __R30 & ~(1 << RM1);
					__R30 = __R30 | (1 << RM0);
				}
				if(received->mstep==3){
					__R30 = __R30 | (1 << LM1);
					__R30 = __R30 & ~(1 << LM0);
					__R30 = __R30 | (1 << RM1);
					__R30 = __R30 & ~(1 << RM0);
				}
				if(received->mstep==4){
					__R30 = __R30 | (1 << LM1);
					__R30 = __R30 | (1 << LM0);
					__R30 = __R30 | (1 << RM1);
					__R30 = __R30 | (1 << RM0);
				}

			}
		}
		for(i=0; i<50000; ++j, ++i, ++k){
			if(j>=received->speedl){
				__R30 = __R30 ^ (1 << LSTEP);
				j=0;
			}
			if(k>=received->speedr){
				__R30 = __R30 ^ (1 << RSTEP);
				k=0;
			}
		}
	}
}
