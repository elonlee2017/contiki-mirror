/*
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
 * This file is part of the Contiki operating system.
 *
 */
/**
 * \file
 *         border-router
 * \author
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 *         Nicolas Tsiftes <nvt@sics.se>
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/uip.h"
#include "net/uip-ds6.h"
#include "net/uip-nd6.h"
#include "net/rpl/rpl.h"

#include "net/netstack.h"
#include "dev/button-sensor.h"
#include "enc28/enc28j60-drv.h"
//#include "webserver-nogui.h"
#include "httpd-simple.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define DEBUG 0
#include "net/uip-debug.h"

uint16_t dag_id[] = {0x1111, 0x1100, 0, 0, 0, 0, 0, 0x0011};

extern uip_ds6_nbr_t uip_ds6_nbr_cache[UIP_DS6_IF_NB][UIP_DS6_NBR_NB];
extern uip_ds6_route_t uip_ds6_routing_table[UIP_DS6_IF_NB][UIP_DS6_ROUTE_NB];

static uip_ipaddr_t prefix;
static uint8_t prefix_set;

PROCESS(border_router_process, "Border router process");
AUTOSTART_PROCESSES(&border_router_process);
/*---------------------------------------------------------------------------*/
/* Only one single web request at time */
static const char *TOP = "<html><head><title>ContikiRPL</title></head><body>\n";
static const char *BOTTOM = "</body></html>\n";
static char buf[128];
static int blen;
#define ADD(...) do {                                                   \
    blen += snprintf(&buf[blen], sizeof(buf) - blen, __VA_ARGS__);      \
  } while(0)
/*---------------------------------------------------------------------------*/
static void
ipaddr_add(const uip_ipaddr_t *addr)
{
  uint16_t a;
  int i, f;
  for(i = 0, f = 0; i < sizeof(uip_ipaddr_t); i += 2) {
    a = (addr->u8[i] << 8) + addr->u8[i + 1];
    if(a == 0 && f >= 0) {
      if(f++ == 0 && sizeof(buf) - blen >= 2) {
        buf[blen++] = ':';
        buf[blen++] = ':';
      }
    } else {
      if(f > 0) {
        f = -1;
      } else if(i > 0 && blen < sizeof(buf)) {
        buf[blen++] = ':';
      }
      ADD("%x", a);
    }
  }
}
/*---------------------------------------------------------------------------*/
static
PT_THREAD(generate_routes(struct httpd_state *s))
{
  static int i;
  PSOCK_BEGIN(&s->sout);

  SEND_STRING(&s->sout, TOP);

  blen = 0;
  ADD("<h2>Neighbors</h2>");
  for(i = 0; i < UIP_DS6_NBR_NB; i++) {
    if(uip_ds6_nbr_cache[IF_RADIO][i].isused) {
      ipaddr_add(&uip_ds6_nbr_cache[IF_RADIO][i].ipaddr);
      ADD("<br>\n");
      if(blen > sizeof(buf) - 45) {
        SEND_STRING(&s->sout, buf);
        blen = 0;
      }
    }
  }

  ADD("<h2>Routes</h2>");
  SEND_STRING(&s->sout, buf);
  blen = 0;
  for(i = 0; i < UIP_DS6_ROUTE_NB; i++) {
    if(uip_ds6_routing_table[IF_RADIO][i].isused) {
      ipaddr_add(&uip_ds6_routing_table[IF_RADIO][i].ipaddr);
      ADD("/%u (via ", uip_ds6_routing_table[IF_RADIO][i].length);
      ipaddr_add(&uip_ds6_routing_table[IF_RADIO][i].nexthop);
      if(uip_ds6_routing_table[IF_RADIO][i].state.lifetime < 600) {
        ADD(") %lus<br>\n", uip_ds6_routing_table[IF_RADIO][i].state.lifetime);
      } else {
        ADD(")<br>\n");
      }
      SEND_STRING(&s->sout, buf);
      blen = 0;
    }
  }
  if(blen > 0) {
    SEND_STRING(&s->sout, buf);
    blen = 0;
  }

  SEND_STRING(&s->sout, BOTTOM);

  PSOCK_END(&s->sout);
}
/*---------------------------------------------------------------------------*/
httpd_simple_script_t
httpd_simple_get_script(const char *name)
{
  return generate_routes;
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Server IPv6 radio addresses:\n");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if[IF_RADIO].addr_list[i].state;
    if(uip_ds6_if[IF_RADIO].addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      blen = 0;
      ipaddr_add(&uip_ds6_if[IF_RADIO].addr_list[i].ipaddr);
      PRINTF("  %s\n", buf);
    }
  }
  PRINTF("Server IPv6 ethernet addresses:\n");
   for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
     state = uip_ds6_if[IF_FALLBACK].addr_list[i].state;
     if(uip_ds6_if[IF_FALLBACK].addr_list[i].isused &&
        (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
       blen = 0;
       ipaddr_add(&uip_ds6_if[IF_FALLBACK].addr_list[i].ipaddr);
       PRINTF("  %s\n", buf);
     }
   }
}

/*---------------------------------------------------------------------------*/
void
set_prefix_64(uip_ipaddr_t *prefix_64, u8_t uip_if_id)
{
  uip_ipaddr_t ipaddr;
  memcpy(&prefix, prefix_64, 16);
  memcpy(&ipaddr, prefix_64, 16);
  prefix_set = 1;
  //Quick hack - RPL must not add prefix to prefix list (addresses matching that prefix
  //will be treated as on-link, and RPL won't work multihop). In current setup Ethernet
  //must add prefix to prefix list, because it has no routing at all.
  if(uip_if_id==IF_FALLBACK)
  {
	  uip_ds6_prefix_add(&prefix, UIP_DEFAULT_PREFIX_LEN, 1,
			  (UIP_ND6_RA_FLAG_ONLINK|UIP_ND6_RA_FLAG_AUTONOMOUS), 600, 600, IF_FALLBACK);
#ifdef UIP_CONF_DS6_ROUTE_INFORMATION
	  prefix.u8[7]=1;
	  uip_ds6_route_info_add(&prefix, 64, 0, 600, IF_FALLBACK);
#endif
  }

  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr[uip_if_id], uip_ds6_if[uip_if_id].lladdr_len);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF, uip_if_id);

}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(border_router_process, ev, data)
{
  static struct etimer et;
  rpl_dag_t *dag;

  PROCESS_BEGIN();
  prefix_set = 0;

  PROCESS_PAUSE();

  //process_start(&webserver_nogui_process, NULL);

  PRINTF("RPL-Border router started\n");

  /* Request prefix until it has been received */
  //while(!prefix_set) {
  //  etimer_set(&et, CLOCK_SECOND);
  //  request_prefix();
  //  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  //}
  uip_ipaddr_t prefix;
  /* Here we set a prefix !!! */
  memset(&prefix, 0, 16);
  prefix.u8[0]=0x20;
  prefix.u8[1]=0x01;
  prefix.u8[2]=0x05;
  prefix.u8[3]=0xc0;
  prefix.u8[4]=0x15;
  prefix.u8[5]=0x15;
  prefix.u8[6]=0x33;
  set_prefix_64(&prefix, IF_FALLBACK);
  prefix.u8[7]=0x01;
  set_prefix_64(&prefix, IF_RADIO);

  dag = rpl_set_root(RPL_DEFAULT_INSTANCE,(uip_ip6addr_t *)dag_id);
  if(dag != NULL) {
    rpl_set_prefix(dag, &prefix, 64);
    PRINTF("created a new RPL dag\n");
  }

  print_local_addresses();

  /* The border router runs with a 100% duty cycle in order to ensure high
     packet reception rates. */
  NETSTACK_MAC.off(1);

  while(1) {
    PROCESS_YIELD();
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
