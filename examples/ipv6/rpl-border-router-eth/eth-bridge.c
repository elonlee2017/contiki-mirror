/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Id: slip-bridge.c,v 1.4 2010/10/21 18:23:44 joxe Exp $
 */

/**
 * \file
 *         Ethernet fallback interface
 * \author
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 *         Joel Hoglund <joel@sics.se>
 *         Nicolas Tsiftes <nvt@sics.se>
 */

#include "net/uip.h"
#include "net/uip-ds6.h"
#include "enc28/enc28j60-drv.h"
#include <string.h>

#define UIP_IP_BUF        ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define DEBUG 0
#include "net/uip-debug.h"

void set_prefix_64(uip_ipaddr_t *);

static uip_ipaddr_t last_sender;

/*---------------------------------------------------------------------------*/
//static void
//eth_input_callback(void)
//{
//  PRINTF("SIN: %u\n", uip_len);
//  /* Save the last sender received over SLIP to avoid bouncing the
//     packet back if no route is found */
//  uip_ipaddr_copy(&last_sender, &UIP_IP_BUF->srcipaddr);
//}
///*---------------------------------------------------------------------------*/
//static void
//init(void)
//{
//	enc28j60_init();
//	process_start(&enc28j60_process, NULL);
//	enc28j60_set_input_callback(eth_input_callback);
//}
/*---------------------------------------------------------------------------*/
//static void
//output(uip_lladdr_t *lladdr)
//{
//  if(uip_ipaddr_cmp(&last_sender, &UIP_IP_BUF->srcipaddr)) {
//    /* Do not bounce packets back over Ethernet if the packet was received
//       over Ethernet */
//    PRINTF("eth-bridge: Destination off-link but no route\n");
//  } else {
//    PRINTF("SUT: %u\n", uip_len);
//    enc28j60_output(lladdr);
//  }
//}

