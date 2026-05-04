/* Networking DHCPv4 client */

/*
 * Copyright (c) 2017 ARM Ltd.
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_dhcpv4_client_sample, LOG_LEVEL_DBG);

#include <zephyr/kernel.h>
#include <zephyr/linker/sections.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/ethernet_mgmt.h>

#define DHCP_OPTION_NTP (42)

static uint8_t ntp_server[4];

static struct net_mgmt_event_callback mgmt_cb;

static struct net_dhcpv4_option_callback dhcp_cb;

static const char *oper_state_to_str(enum net_if_oper_state state)
{
	switch (state) {
	case NET_IF_OPER_UNKNOWN:
		return "UNKNOWN";
	case NET_IF_OPER_NOTPRESENT:
		return "NOTPRESENT";
	case NET_IF_OPER_DOWN:
		return "DOWN";
	case NET_IF_OPER_LOWERLAYERDOWN:
		return "LOWERLAYERDOWN";
	case NET_IF_OPER_TESTING:
		return "TESTING";
	case NET_IF_OPER_DORMANT:
		return "DORMANT";
	case NET_IF_OPER_UP:
		return "UP";
	default:
		return "INVALID";
	}
}

static void log_iface_status(struct net_if *iface, const char *tag)
{
	enum net_if_oper_state oper = net_if_oper_state(iface);

	LOG_INF("%s: iface=%s idx=%d is_up=%d admin=%d carrier=%d oper=%s(%u)",
		tag,
		net_if_get_device(iface)->name,
		net_if_get_by_iface(iface),
		net_if_is_up(iface),
		net_if_is_admin_up(iface),
		net_if_is_carrier_ok(iface),
		oper_state_to_str(oper),
		(unsigned int)oper);
}

static bool is_dm9051_iface(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);

	if (dev == NULL || dev->name == NULL) {
		return false;
	}

	return (strstr(dev->name, "dm9051") != NULL) ||
	       (strstr(dev->name, "DM9051") != NULL);
}

static void start_dhcpv4_client(struct net_if *iface, void *user_data)
{
	int ret;

	ARG_UNUSED(user_data);
	log_iface_status(iface, "Before net_if_up");

	ret = net_if_up(iface);
	if (ret < 0 && ret != -EALREADY) {
		LOG_WRN("Failed to bring iface %s up (%d)",
			net_if_get_device(iface)->name, ret);
	}
	log_iface_status(iface, "After net_if_up");

	LOG_INF("Start on %s: index=%d", net_if_get_device(iface)->name,
		net_if_get_by_iface(iface));
	LOG_INF("Calling net_dhcpv4_start() on iface=%s", net_if_get_device(iface)->name);
	net_dhcpv4_start(iface);
	log_iface_status(iface, "After net_dhcpv4_start");
}

static void handler(struct net_mgmt_event_callback *cb,
		    uint64_t mgmt_event,
		    struct net_if *iface)
{
	int i = 0;

	ARG_UNUSED(cb);

	if (mgmt_event == NET_EVENT_IF_UP) {
		LOG_INF("NET_EVENT_IF_UP");
		log_iface_status(iface, "IF_UP");
		return;
	}

	if (mgmt_event == NET_EVENT_IF_DOWN) {
		LOG_INF("NET_EVENT_IF_DOWN");
		log_iface_status(iface, "IF_DOWN");
		return;
	}

	if (mgmt_event == NET_EVENT_ETHERNET_CARRIER_ON) {
		if (is_dm9051_iface(iface)) {
			LOG_INF("DM9051 link up (iface=%s, index=%d)",
				net_if_get_device(iface)->name,
				net_if_get_by_iface(iface));
		}
		log_iface_status(iface, "CARRIER_ON");
		return;
	}

	if (mgmt_event == NET_EVENT_ETHERNET_CARRIER_OFF) {
		if (is_dm9051_iface(iface)) {
			LOG_INF("DM9051 link down (iface=%s, index=%d)",
				net_if_get_device(iface)->name,
				net_if_get_by_iface(iface));
		}
		log_iface_status(iface, "CARRIER_OFF");
		return;
	}

	if (mgmt_event == NET_EVENT_IPV4_DHCP_START) {
		LOG_INF("NET_EVENT_IPV4_DHCP_START");
		log_iface_status(iface, "DHCP_START");
		return;
	}

	if (mgmt_event == NET_EVENT_IPV4_DHCP_BOUND) {
		LOG_INF("NET_EVENT_IPV4_DHCP_BOUND lease=%u", iface->config.dhcpv4.lease_time);
		log_iface_status(iface, "DHCP_BOUND");
		return;
	}

	if (mgmt_event == NET_EVENT_IPV4_DHCP_STOP) {
		LOG_INF("NET_EVENT_IPV4_DHCP_STOP");
		log_iface_status(iface, "DHCP_STOP");
		return;
	}

	if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD) {
		return;
	}

	for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
		char buf[NET_IPV4_ADDR_LEN];

		if (iface->config.ip.ipv4->unicast[i].ipv4.addr_type !=
							NET_ADDR_DHCP) {
			continue;
		}

		LOG_INF("   Address[%d]: %s", net_if_get_by_iface(iface),
			net_addr_ntop(NET_AF_INET,
			    &iface->config.ip.ipv4->unicast[i].ipv4.address.in_addr,
						  buf, sizeof(buf)));
		LOG_INF("    Subnet[%d]: %s", net_if_get_by_iface(iface),
			net_addr_ntop(NET_AF_INET,
				       &iface->config.ip.ipv4->unicast[i].netmask,
				       buf, sizeof(buf)));
		LOG_INF("    Router[%d]: %s", net_if_get_by_iface(iface),
			net_addr_ntop(NET_AF_INET,
						 &iface->config.ip.ipv4->gw,
						 buf, sizeof(buf)));
		LOG_INF("Lease time[%d]: %u seconds", net_if_get_by_iface(iface),
			iface->config.dhcpv4.lease_time);
	}
}

static void option_handler(struct net_dhcpv4_option_callback *cb,
			   size_t length,
			   enum net_dhcpv4_msg_type msg_type,
			   struct net_if *iface)
{
	char buf[NET_IPV4_ADDR_LEN];

	LOG_INF("DHCP Option %d: %s", cb->option,
		net_addr_ntop(NET_AF_INET, cb->data, buf, sizeof(buf)));
}

int main(void)
{
	//LOG_INF("Run dhcpv4 client");
	LOG_INF("Run dhcpv4 client - use 'west build -t menuconfig' to set the correct MAC address if it is not set in device tree or if random MAC address is not used");
	LOG_INF("Run dhcpv4 client - make sure to connect the Ethernet cable ");
	LOG_INF("Run dhcpv4 client - before starting the board ");

	net_mgmt_init_event_callback(&mgmt_cb, handler,
				     NET_EVENT_IF_UP |
				     NET_EVENT_IF_DOWN |
				     NET_EVENT_IPV4_ADDR_ADD |
				     NET_EVENT_IPV4_DHCP_START |
				     NET_EVENT_IPV4_DHCP_BOUND |
				     NET_EVENT_IPV4_DHCP_STOP |
				     NET_EVENT_ETHERNET_CARRIER_ON |
				     NET_EVENT_ETHERNET_CARRIER_OFF);
	net_mgmt_add_event_callback(&mgmt_cb);

	net_dhcpv4_init_option_callback(&dhcp_cb, option_handler,
					DHCP_OPTION_NTP, ntp_server,
					sizeof(ntp_server));

	net_dhcpv4_add_option_callback(&dhcp_cb);

	net_if_foreach(start_dhcpv4_client, NULL);
	return 0;
}
