/* DM9058 Stand-alone Ethernet Controller with SPI
 *
 * Copyright (c) 2025~2026 Davicom Semiconductor Incorporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT davicom_dm9058

#define LOG_MODULE_NAME eth_dm9058
#define LOG_LEVEL       CONFIG_ETHERNET_LOG_LEVEL

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/random/random.h>
#include <string.h>
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#if defined(CONFIG_PTP_CLOCK) || defined(CONFIG_NET_GPTP)
#include <zephyr/drivers/ptp_clock.h>
#endif
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/toolchain.h>
#include <ethernet/eth_stats.h>

#include "eth_dm9058_priv.h"

static uint8_t dm9058_read_reg(const struct device *dev, uint8_t reg);
static void dm9058_read_mem(const struct device *dev, uint8_t *buf, uint16_t len);
struct net_ptp_time;
int dm9058_ptp_read_time_locked(const struct device *eth_dev, struct net_ptp_time *tm);

#ifdef CONFIG_ETH_DM9058_DEBUG_PRINTS
#define DM9058_DBG(...) printk(__VA_ARGS__)
int drvc;
#define DM9058_ENDC_INC() (drvc++)
#define DM9058_ENDC_GET() (drvc)
#define DM9058_ENDC_SAVE(sav) int sav = DM9058_ENDC_INC()
#define DM9058_ENDC_RETRIVE(sav) sav
#else
#define DM9058_DBG(...) do { } while (0)
#define DM9058_ENDC_INC() (0)
#define DM9058_ENDC_GET() (0)
#define DM9058_ENDC_SAVE(sav)
#define DM9058_ENDC_RETRIVE(sav)
#endif

/**
 * @brief Drop packet from DM9058's memory to prevent blocking
 * @param dev Device structure
 * @param rx_len Length of packet to discard (including padding)
 */
static void dm9058_drop_packet(const struct device *dev, uint16_t rx_len)
{
	/* Avoid VLA/large stack usage: discard in bounded chunks. */
	uint8_t dummy[32];
	uint16_t remaining = rx_len;

	while (remaining > 0) {
		uint16_t chunk = MIN(remaining, (uint16_t)sizeof(dummy));

		dm9058_read_mem(dev, dummy, chunk);
		remaining -= chunk;
	}
	
	/* MBNDRY_DEFAULT - Pad to even length */
	if (rx_len & 1) {
		uint8_t pad;
		dm9058_read_mem(dev, &pad, 1);
	}
}

static bool dm9058_mac_is_valid(const uint8_t mac[6])
{
 bool all_zero = true;

 for (int i = 0; i < 6; i++) {
  if (mac[i] != 0x00) {
   all_zero = false;
   break;
  }
 }

 if (all_zero) {
  return false;
 }

 /* Reject multicast/broadcast */
 if ((mac[0] & 0x01) != 0) {
  return false;
 }

 return true;
}

static void dm9058_generate_random_mac(uint8_t mac[6])
{
	uint32_t r = sys_rand32_get();

	/* OUI: Davicom vendor prefix */
	mac[0] = 0x00;
	mac[1] = 0x60;
	mac[2] = 0x6e;

	/* NIC: random bytes */
	mac[3] = (uint8_t)(r >> 16);
	mac[4] = (uint8_t)(r >> 8);
	mac[5] = (uint8_t)(r & 0xFF);
}

int dm9058_load_mac_from_current_fit(const struct device *dev, uint8_t mac[6])
{
 for (int i = 0; i < 6; i++) {
  mac[i] = dm9058_read_reg(dev, DM9058_PAR + i);
 }
 return 0;
}

int dm9058_read_mem_cb(void *ctx, uint8_t *buf, int len)
{
	const struct device *dev = ctx;

	dm9058_read_mem(dev, buf, len);
	return 0;
}

/*
 * The following function and type definition are local implementations of
 * APIs that may not be available in the user's SDK version. This ensures
 * compatibility while using modern Zephyr patterns.
 */
typedef int (*net_pkt_read_from_cb_t)(void *ctx, uint8_t *buf, int len);

static inline int local_net_pkt_read_from(struct net_pkt *pkt, net_pkt_read_from_cb_t cb,
					   void *ctx, size_t len)
{
	size_t remaining = len;
	struct net_buf *frag = pkt->buffer;

	while (frag && remaining > 0) {
		size_t copy_len = MIN(remaining, net_buf_tailroom(frag));
		int ret;

		ret = cb(ctx, net_buf_add(frag, copy_len), copy_len);
		if (ret < 0) {
			return ret;
		}

		remaining -= copy_len;

		if (remaining > 0) {
			frag = frag->frags;
		}
	}

	if (remaining > 0) {
		return -ENOMEM;
	}

	return 0;
}

/* Driver configuration structure */
struct driver_config {
	const char *release_version;
};

/* Default driver configuration */
const struct driver_config confdata = {
	.release_version = "zephyr_dm9058_v3.1.0_v1.0",
};

/* Helper macro to check if interrupt mode is enabled based on device tree configuration */
#define crst(dev) (((const struct dm9058_config *)(dev)->config)->reset.port != NULL)
#define cint(dev) (((const struct dm9058_config *)(dev)->config)->interrupt.port != NULL)

/* DM9058 Constants */
#define DM9058_PHY     (0x40)
#define DM9058_PKT_RDY (0x01)
#define PHY_ADV_REG    (0x04)

/*******************************************************************************
 * Hardware Abstraction Layer - SPI Operations
 ******************************************************************************/

/* Note: All SPI operations use spi_transceive_dt() or spi_write_dt() directly.
 * CS (Chip Select) is automatically controlled by the SPI driver layer.
 * The cs-gpios property in device tree specifies which GPIO pin to use for CS.
 * We use static buffers on stack (like W5500) instead of k_malloc.
 */

/**
 * @brief Read single register from DM9058
 * @param dev Device structure
 * @param reg Register address
 * @return Register value
 */
static uint8_t dm9058_read_reg(const struct device *dev, uint8_t reg)
{
	const struct dm9058_config *config = dev->config;
	uint8_t tx_data[2] = {reg | OPC_REG_R, 0x00};
	uint8_t rx_data[2] = {0};

	struct spi_buf tx_buf = {.buf = tx_data, .len = 2};
	struct spi_buf rx_buf = {.buf = rx_data, .len = 2};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

	int ret = spi_transceive_dt(&config->spi, &tx, &rx);
	if (ret < 0) {
		LOG_ERR("SPI read register failed: %d", ret);
		return 0xFF;
	}

	return rx_data[1];
}

/**
 * @brief Write single register to DM9058
 * @param dev Device structure
 * @param reg Register address
 * @param val Value to write
 */
static void dm9058_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct dm9058_config *config = dev->config;
	uint8_t tx_data[2] = {reg | OPC_REG_W, val};

	struct spi_buf tx_buf = {.buf = tx_data, .len = 2};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

	int ret = spi_write_dt(&config->spi, &tx);
	if (ret < 0) {
		LOG_ERR("SPI write register failed: %d", ret);
	}
}

/**
 * @brief Read multiple bytes from DM9058 memory
 * @param dev Device structure
 * @param buf Buffer to store read data
 * @param len Number of bytes to read
 */
static void dm9058_read_mem(const struct device *dev, uint8_t *buf, uint16_t len)
{
	const struct dm9058_config *config = dev->config;
	uint8_t cmd = DM9058_MRCMD | OPC_REG_R;

	const struct spi_buf tx_buf = {.buf = &cmd, .len = 1};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

	const struct spi_buf rx_buf[2] = {
		{.buf = NULL, .len = 1}, /* Discard command echo */
		{.buf = buf, .len = len} /* Actual data */
	};
	const struct spi_buf_set rx = {.buffers = rx_buf, .count = 2};

	int ret = spi_transceive_dt(&config->spi, &tx, &rx);
	if (ret < 0) {
		LOG_ERR("SPI read memory failed: %d", ret);
	}
}

/**
 * @brief Write multiple bytes to DM9058 memory
 * @param dev Device structure
 * @param buf Buffer containing data to write
 * @param len Number of bytes to write
 */
static void dm9058_write_mem(const struct device *dev, const uint8_t *buf, uint16_t len)
{
	const struct dm9058_config *config = dev->config;
	uint8_t cmd = DM9058_MWCMD | OPC_REG_W;

	const struct spi_buf tx_buf[2] = {
		{.buf = &cmd, .len = 1},         /* Command byte */
		{.buf = (void *)buf, .len = len} /* Data bytes */
	};
	const struct spi_buf_set tx = {.buffers = tx_buf, .count = 2};

	int ret = spi_write_dt(&config->spi, &tx);
	if (ret < 0) {
		LOG_ERR("SPI write memory failed: %d", ret);
	}
}

/*******************************************************************************
 * PHY Operations
 ******************************************************************************/

/**
 * @brief Read PHY register
 * @param dev Device structure
 * @param reg PHY register address
 * @return PHY register value
 */
#if 0
static uint16_t dm9058_phy_read(const struct device *dev, uint16_t reg)
{
	uint16_t value;
	int timeout = 500;

	dm9058_write_reg(dev, DM9058_EPAR, DM9058_PHY | reg);
	dm9058_write_reg(dev, DM9058_EPCR, 0x0c);
	k_busy_wait(1);

	while ((dm9058_read_reg(dev, DM9058_EPCR) & 0x01) && timeout--) {
		k_busy_wait(1);
	}

	if (dm9058_read_reg(dev, DM9058_EPCR) & 0x01) {
		return 0xffff;
	}

	dm9058_write_reg(dev, DM9058_EPCR, 0x00);
	value = (dm9058_read_reg(dev, DM9058_EPDRH) << 8) | dm9058_read_reg(dev, DM9058_EPDRL);

	return value;
}
#endif

/**
 * @brief Write PHY register
 * @param dev Device structure
 * @param reg PHY register address
 * @param value Value to write
 */
static void dm9058_phy_write(const struct device *dev, uint16_t reg, uint16_t value)
{
	int timeout = 500;

	dm9058_write_reg(dev, DM9058_EPAR, DM9058_PHY | reg);
	dm9058_write_reg(dev, DM9058_EPDRL, value & 0xff);
	dm9058_write_reg(dev, DM9058_EPDRH, (value >> 8) & 0xff);
	dm9058_write_reg(dev, DM9058_EPCR, 0x0a);
	k_busy_wait(1);

	while ((dm9058_read_reg(dev, DM9058_EPCR) & 0x01) && timeout--) {
		k_busy_wait(1);
	}

	dm9058_write_reg(dev, DM9058_EPCR, 0x00);
}

void dm9058_interrupt_disble_irq(const struct device *dev)
{
	dm9058_write_reg(dev, DM9058_IMR, IMR_PAR);
}

void dm9058_isr_enab(const struct device *dev)
{
	uint8_t isrs = dm9058_read_reg(dev, DM9058_ISR);
	dm9058_write_reg(dev, DM9058_ISR, isrs);
}
void dm9058_imr_enab(const struct device *dev)
{
	dm9058_write_reg(dev, DM9058_IMR, IMR_INT_DEFAULT);
}

static void dm9058_interrupt_reset_for_cb_sem(const struct device *dev)
{
	dm9058_isr_enab(dev);
	dm9058_imr_enab(dev);
}

/*******************************************************************************
 * Core Driver Functions
 ******************************************************************************/

/**
 * @brief Perform core reset of DM9058
 * @param dev Device structure
 */
static void dm9058_core_reset(const struct device *dev)
{
	/* Power on PHY */
	dm9058_write_reg(dev, DM9058_GPR, 0x00);
	k_msleep(25);

	/* NCR reset */
	dm9058_write_reg(dev, DM9058_NCR, DM9058_NCR_RESET);
	k_msleep(5);

	/* Wait for reset completion */
	int timeout = 100;
	while ((dm9058_read_reg(dev, DM9058_NCR) & DM9058_NCR_RESET) && timeout--) {
		k_msleep(1);
	}

	/* Software defaults */
	dm9058_write_reg(dev, DM9058_MBNDRY, MBNDRY_DEFAULT);
	dm9058_write_reg(dev, DM9058_PPCR, PPCR_PAUSE_COUNT);
	dm9058_write_reg(dev, DM9058_LMCR, LMCR_MODE1);
	dm9058_write_reg(dev, DM9058_INTR, INTR_ACTIVE_LOW);

#ifdef CONFIG_ETH_DM9058_TX_CHECKSUM_OFFLOAD
	/* Enable TX checksum offload */
	dm9058_write_reg(dev, DM9058_CSCR,
			 TCSCR_IPCS_ENABLE | TCSCR_UDPCS_ENABLE | TCSCR_TCPCS_ENABLE);
#endif

#ifdef CONFIG_ETH_DM9058_RX_CHECKSUM_OFFLOAD
	/* Enable RX checksum offload */
	dm9058_write_reg(dev, DM9058_RCSSR, RCSSR_RCSEN | RCSSR_DCSE);
#endif

	LOG_DBG("%s: Core reset complete", dev->name);
}

/**
 * @brief Get chip ID
 * @param dev Device structure
 * @return Chip ID for supported compatible variants
 */
static uint16_t dm9058_get_chipid(const struct device *dev)
{
	uint16_t id;
	uint8_t pidh, pidl;

	pidh = dm9058_read_reg(dev, DM9058_PIDH);
	pidl = dm9058_read_reg(dev, DM9058_PIDL);
	id = (pidh << 8) | pidl;

	/* Print raw register values for debugging */
	// LOG_INF("DEBUG: Chip ID registers - PIDH: 0x%02x, PIDL: 0x%02x, Combined: 0x%04x", pidh,
	//	pidl, id);

	/* Legacy chips may report 0x9000; normalize to internal compatible ID. */
	if (id == 0x9000) {
		id = 0x9051;
		LOG_INF("DEBUG: Normalized chip ID from 0x9000 to internal compatible ID");
	}

	return id;
}

/**
 * @brief Set MAC address
 * @param dev Device structure
 * @param mac MAC address array (6 bytes)
 */
static void dm9058_set_mac_address(const struct device *dev, const uint8_t *mac)
{
	for (int i = 0; i < 6; i++) {
		dm9058_write_reg(dev, DM9058_PAR + i, mac[i]);
	}
}

/**
 * @brief Configure multicast address registers
 * @param dev Device structure
 */
static void dm9058_set_multicast(const struct device *dev)
{
	for (int i = 0; i < 8; i++) {
		dm9058_write_reg(dev, DM9058_MAR + i, (i == 7) ? 0x80 : 0x00);
	}
}

static uint8_t dm9058_multicast_hash6(const uint8_t *mac)
{
	uint32_t crc = 0xFFFFFFFFU;

	for (int j = 0; j < 6; j++) {
		crc ^= mac[j];
		for (int i = 0; i < 8; i++) {
			if (crc & 1U) {
				crc = (crc >> 1) ^ 0xEDB88320U;
			} else {
				crc >>= 1;
			}
		}
	}

	return (uint8_t)(crc & 0x3FU);
}

static void dm9058_allow_multicast_mac(const struct device *dev, const uint8_t mac[6])
{
	uint8_t mar[8];
	uint8_t hash = dm9058_multicast_hash6(mac);
	uint8_t mar_idx = hash / 8U;
	uint8_t bit_idx = hash % 8U;

	for (int i = 0; i < 8; i++) {
		mar[i] = dm9058_read_reg(dev, DM9058_MAR + i);
	}

	mar[mar_idx] |= BIT(bit_idx);

	for (int i = 0; i < 8; i++) {
		dm9058_write_reg(dev, DM9058_MAR + i, mar[i]);
	}

	LOG_INF("%s: allow multicast %02x:%02x:%02x:%02x:%02x:%02x hash=%u mar[%u].bit%u",
		dev->name,
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
		hash, mar_idx, bit_idx);
}

/**
 * @brief Configure receive settings
 * @param dev Device structure
 */
static void dm9058_set_receive(const struct device *dev)
{
	uint8_t rcr = RCR_DEFAULT | RCR_RXEN;

#if 0
	/* Configure multicast addresses */
	dm9058_set_multicast(dev);
#endif
	/* Configure flow control */
	dm9058_write_reg(dev, DM9058_FCR, FCR_DEFAULT);
	dm9058_phy_write(dev, PHY_ADV_REG, 0x0400 | 0x01e1);

	/* Configure interrupts based on device tree configuration */
	if (cint(dev)) {
		/* Interrupt mode enabled via int-gpios in device tree */
		dm9058_write_reg(dev, DM9058_IMR, IMR_INT_DEFAULT);
	} else {
		/* Polling mode (no int-gpios defined) */
		dm9058_write_reg(dev, DM9058_IMR, IMR_POL_DEFAULT);
	}

	/* In gPTP bring-up, accept all multicast to avoid PTP destination filtering. */
#if defined(CONFIG_NET_GPTP)
	rcr |= RCR_ALL;
#endif

	/* Enable receiver */
	dm9058_write_reg(dev, DM9058_RCR, rcr);

#if defined(CONFIG_NET_GPTP)
	/* gPTP in Zephyr uses fixed L2 multicast destination 01:80:C2:00:00:0E. */
	static const uint8_t gptp_l2_addr[6] = { 0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E };
	dm9058_allow_multicast_mac(dev, gptp_l2_addr);
#endif

	LOG_DBG("%s: Receive configured (%s mode)", dev->name, cint(dev) ? "INTERRUPT" : "POLLING");
}

#if defined(CONFIG_NET_GPTP)
/* DM9058 RX status bits multiplex timestamp metadata in PTP mode. */
#define DM9058_RSR_RXTS_EN     BIT(5)
#define DM9058_RSR_RXTS_PARITY BIT(3)
#define DM9058_RSR_RXTS_LEN    BIT(2)
#define DM9058_RSR_PTP_BITS (DM9058_RSR_RXTS_EN | DM9058_RSR_RXTS_PARITY | DM9058_RSR_RXTS_LEN)

static inline uint8_t dm9058_rx_error_mask(void)
{
	/* Follow ESP-IDF logic: do not treat timestamp status bits as RX errors. */
	return (uint8_t)(RSR_ERR_BITS & ~DM9058_RSR_PTP_BITS);
}

static inline size_t dm9058_rx_timestamp_len(uint8_t rx_status)
{
	if ((rx_status & DM9058_RSR_RXTS_EN) == 0U) {
		return 0U;
	}

	return (rx_status & DM9058_RSR_RXTS_LEN) ? 8U : 4U;
}

static inline void dm9058_decode_rx_timestamp(const uint8_t *buf, size_t len,
					      struct net_ptp_time *ts)
{
	/* Format is aligned with ESP-IDF DM9058 implementation. */
	ts->nanosecond = (uint32_t)buf[7] |
			 ((uint32_t)buf[6] << 8) |
			 ((uint32_t)buf[5] << 16) |
			 ((uint32_t)buf[4] << 24);

	if (len == 8U) {
		ts->second = (uint32_t)buf[3] |
			     ((uint32_t)buf[2] << 8) |
			     ((uint32_t)buf[1] << 16) |
			     ((uint32_t)buf[0] << 24);
	} else {
		ts->second = 0U;
	}
}
#else
static inline uint8_t dm9058_rx_error_mask(void)
{
	return RSR_ERR_BITS;
}
#endif

#if defined(CONFIG_NET_GPTP)
#ifndef DM9058_PTPCW
#define DM9058_PTPCW              0x61
#endif
#ifndef DM9058_PTPTS
#define DM9058_PTPTS              0x68
#endif
#ifndef DM9058_PTP_TCR_READ_CLOCK
#define DM9058_PTP_TCR_READ_CLOCK 0x84
#endif
#ifndef TCR_TSEN_CAP
#define TCR_TSEN_CAP TCR_RSV_BIT7  /* TX timestamp capture enable (two-step PTP) */
#endif
#ifndef DM9058_PTPCW2
#define DM9058_PTPCW2             0x62  /* PTP TX timestamp mode select */
#endif

int dm9058_ptp_read_time_locked(const struct device *eth_dev, struct net_ptp_time *tm)
{
	uint8_t raw[8] = {0};

	if (tm == NULL) {
		return -EINVAL;
	}

	dm9058_write_reg(eth_dev, DM9058_PTPCW, DM9058_PTP_TCR_READ_CLOCK);
	for (size_t i = 0; i < sizeof(raw); i++) {
		raw[i] = dm9058_read_reg(eth_dev, DM9058_PTPTS);
	}

	tm->nanosecond = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) |
			 ((uint32_t)raw[2] << 16) | ((uint32_t)raw[3] << 24);
	tm->second = (uint32_t)raw[4] | ((uint32_t)raw[5] << 8) |
		     ((uint32_t)raw[6] << 16) | ((uint32_t)raw[7] << 24);

	return 0;
}

/* Read TX timestamp captured by hardware after TCR_TSEN_CAP transmission.
 * Follows impl_dm9051_get_tx_timestamp:
 *   1. Write 0x80 to PTPCW  (0x61) — reset timestamp read index
 *   2. Write 0x01 to PTPCW2 (0x62) — select TX timestamp read mode
 *   3. Read 8 bytes from PTPTS (0x68)
 */
static int dm9058_ptp_read_tx_timestamp(const struct device *dev, struct net_ptp_time *tm)
{
	uint8_t raw[8] = {0};

	if (tm == NULL) {
		return -EINVAL;
	}

	dm9058_write_reg(dev, DM9058_PTPCW,  0x80); /* reset index */
	dm9058_write_reg(dev, DM9058_PTPCW2, 0x01); /* TX timestamp read mode */
	for (size_t i = 0; i < sizeof(raw); i++) {
		raw[i] = dm9058_read_reg(dev, DM9058_PTPTS);
	}

	tm->nanosecond = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) |
			 ((uint32_t)raw[2] << 16) | ((uint32_t)raw[3] << 24);
	tm->second = (uint32_t)raw[4] | ((uint32_t)raw[5] << 8) |
		     ((uint32_t)raw[6] << 16) | ((uint32_t)raw[7] << 24);

	return 0;
}

/* Mirror is_issue_ptp_tstamp_tsen for Zephyr L2 gPTP frames (EtherType 0x88F7).
 * Returns true for PTP message types requiring hardware TX timestamp capture:
 *   msgtype 0x00 SYNC      (two-step): TX ts goes into Follow_Up
 *   msgtype 0x01 DELAY_REQ           : TX ts used as T3
 *   msgtype 0x02 PDELAY_REQ          : TX ts used as T3 in peer-delay
 * Mirrors is_issue_ptp_tstamp_tsen:
 *   (is_ptp_sync_packet && TWO_STEP) || is_ptp_delayreq || is_ptp_pdelayreq
 */
static inline bool dm9058_ptp_needs_tx_ts(struct net_pkt *pkt)
{
	const struct net_buf *buf = pkt->frags;
	uint8_t msgtype;

	if (!buf || buf->len < 15U) {
		return false;
	}

	/* EtherType 0x88F7 at offset 12 */
	if (sys_get_be16(&buf->data[12]) != 0x88F7U) {
		return false;
	}

	/* messageType: lower nibble of PTP header byte 0 (offset 14) */
	msgtype = buf->data[14] & 0x0FU;

	return (msgtype == 0x00U   /* SYNC two-step */
		|| msgtype == 0x01U  /* DELAY_REQ */
		|| msgtype == 0x02U);/* PDELAY_REQ */
}
#endif

/*******************************************************************************
 * Packet Transmission
 ******************************************************************************/

/**
 * @brief Transmit packet
 * @param dev Device structure
 * @param pkt Network packet
 * @return 0 on success, negative errno on failure
 */
static int eth_dm9058_tx(const struct device *dev, struct net_pkt *pkt)
{
	struct dm9058_runtime *context = dev->data;
	uint16_t len = net_pkt_get_len(pkt);
	struct net_buf *frag;
	int timeout = 500;
	uint8_t tcr_wr = TCR_TXREQ;
#if defined(CONFIG_PTP_CLOCK) && defined(CONFIG_NET_GPTP)
	struct net_ptp_time tx_ts;
	bool tx_ts_valid = false;
#endif

	LOG_DBG("%s: TX packet len=%u", dev->name, len);

	k_sem_take(&context->tx_rx_sem, K_FOREVER);

	/* tx pad default_boundary */
	//uint16_t pad_len = (MBNDRY_DEFAULT == MBNDRY_WORD) && (len & 1) ? len + 1 : len;

	/* Set packet length */
	dm9058_write_reg(dev, DM9058_TXPLL, len & 0xff);
	dm9058_write_reg(dev, DM9058_TXPLH, (len >> 8) & 0xff);

	/* Write packet data */
	//uint16_t pad = 0;
	for (frag = pkt->frags; frag; frag = frag->frags) {
		//if ((MBNDRY_DEFAULT == MBNDRY_WORD) && !frag->frags && (frag->len & 1))
		//	pad = 1;
		dm9058_write_mem(dev, frag->data, frag->len); // + pad
	}

	/* MBNDRY_DEFAULT */
	/* Pad to even length */
	if (len & 1) {
		uint8_t pad = 0x00;
		dm9058_write_mem(dev, &pad, 1);
	}

#if defined(CONFIG_PTP_CLOCK) && defined(CONFIG_NET_GPTP)
	/* Mirror ptp_tx_tstamp_parse_packet: set TCR_TSEN_CAP for PTP packets
	 * so the hardware captures the TX timestamp.
	 */
	/* If.Joseph: In a more modern SDK with better net_pkt parsing support, we could have a cleaner check here.
	 * For example, if an API like
	 *   if (net_pkt_is_ptp(pkt)) { ... }
	 *   if (dm9058_ptp_needs_tx_ts(pkt)) { ... } MCU's.
	 * is available in the user's SDK, it can be used here instead of dm9058
	 */
	if (net_pkt_is_ptp(pkt)) {
		tcr_wr |= TCR_TSEN_CAP;
	}
#endif

	/* Trigger transmission */
	dm9058_write_reg(dev, DM9058_TCR, tcr_wr);

	/* Wait for completion with timeout */
	while (timeout--) {
		if (!(dm9058_read_reg(dev, DM9058_TCR) & TCR_TXREQ)) {
			break;
		}
		k_busy_wait(1);
	}

#if defined(CONFIG_PTP_CLOCK) && defined(CONFIG_NET_GPTP)
	/* Mirror ptp_tx_tstamp_pass_to: read captured TX timestamp only
	 * when TCR_TSEN_CAP was set (impl_dm9051_get_tx_timestamp pattern).
	 */
	if (timeout > 0 && (tcr_wr & TCR_TSEN_CAP)) {
		if (dm9058_ptp_read_tx_timestamp(dev, &tx_ts) == 0) {
			net_pkt_set_timestamp(pkt, &tx_ts);
			tx_ts_valid = true;
		}
	}
#endif

	k_sem_give(&context->tx_rx_sem);

	if (timeout == 0) {
		LOG_ERR("%s: TX timeout", dev->name);
		return -ETIMEDOUT;
	}

#if defined(CONFIG_PTP_CLOCK) && defined(CONFIG_NET_GPTP)
	if (tx_ts_valid) {
		net_if_add_tx_timestamp(pkt);
	}
#endif

	LOG_DBG("%s: TX successful", dev->name);
	return 0;
}

/*******************************************************************************
 * Packet Reception
 ******************************************************************************/

/**
 * @brief  Variable argument error handler with reset
 *
 * @param  format   Error format string
 * @param  ...      Variable arguments
 * @return          0 after reset completion
 */
void env_err_rst(const struct device *dev)
{
  dm9058_core_reset(dev); //cspi_core_reset();
  dm9058_set_receive(dev); //cspi_core_start1();
}

/**
 * @brief Check if RX packet is ready
 * @param dev Device structure
 * @return true if packet ready, false otherwise
 */
static bool dm9058_rx_ready(const struct device *dev)
{
	uint8_t rxbyte;

	/* Read RX byte twice (dummy read first) */
	rxbyte = dm9058_read_reg(dev, DM9058_MRCMDX);
	rxbyte = dm9058_read_reg(dev, DM9058_MRCMDX);

	return (rxbyte & 0x01) == DM9058_PKT_RDY;
}

/**
 * @brief Receive packet
 * @param dev Device structure
 * @return 0 on success, negative errno on failure
 */
static int dm9058_rx_packet(const struct device *dev)
{
	const struct dm9058_config *config = dev->config;
	struct dm9058_runtime *context = dev->data;
	uint8_t header[4];
	uint16_t rx_len;
	uint8_t rx_status;
	struct net_pkt *pkt;
#if defined(CONFIG_PTP_CLOCK) && defined(CONFIG_NET_GPTP)
	struct net_ptp_time rx_ts;
	//struct net_ptp_time phc_ts;
	//bool rx_phc_valid = false;
	bool rx_ts_valid = false;
	uint8_t rx_ts_buf[8] = {0};
	size_t rx_ts_len = 0;
#endif

	if (!dm9058_rx_ready(dev)) {
		return 1; // 0;
	}

	/* Read packet header */
	dm9058_read_mem(dev, header, 4);
	dm9058_write_reg(dev, DM9058_ISR, 0x80);

	rx_status = header[1];
	rx_len = header[2] | (header[3] << 8);

	/* Validate packet */
	if (rx_status & dm9058_rx_error_mask()) {
		LOG_ERR("%s: RX error status=0x%02x", dev->name, rx_status);
		env_err_rst(dev);
		return -EIO;
	}

	if (rx_len > NET_ETH_MTU + sizeof(struct net_eth_hdr) + 4 || rx_len < 4) {
		LOG_ERR("%s: RX length error len=%u", dev->name, rx_len);
		env_err_rst(dev);
		return -EINVAL;
	}

	/* rx_len default_boundary */
	//if (MBNDRY_DEFAULT == MBNDRY_WORD)
	//	rx_len = ((rx_len + 1) >> 1) << 1; 


	/* rx_len from chip includes 4-byte CRC, but net_pkt is for frame data only */
	uint16_t frame_len = rx_len - 4;

#if defined(CONFIG_PTP_CLOCK) && defined(CONFIG_NET_GPTP)
	rx_ts_len = dm9058_rx_timestamp_len(rx_status);
	if (rx_ts_len > 0U) {
		dm9058_read_mem(dev, rx_ts_buf, rx_ts_len);
		dm9058_decode_rx_timestamp(rx_ts_buf, rx_ts_len, &rx_ts);
		rx_ts_valid = true;
	}
#endif

	/* Allocate packet buffer for the frame with timeout retry strategy
	 * First attempt: use configured timeout
	 * If that fails, try with K_NO_WAIT in case buffers become available
	 */
	pkt = net_pkt_rx_alloc_with_buffer(context->iface, frame_len, AF_UNSPEC, 0,
					   K_MSEC(config->timeout_pkt)); //K_MSEC(config->timeout)
	if (!pkt) {
		/* Retry once without blocking - buffers may have been freed by RX thread */
		pkt = net_pkt_rx_alloc_with_buffer(context->iface, frame_len, AF_UNSPEC, 0,
						   K_NO_WAIT);
		if (!pkt) {
			LOG_WRN("%s: RX buffer allocation failed (size=%u, available pools: "
				"RX_PKT=%d, RX_BUF=%d) - discarding packet",
				dev->name, frame_len,
				CONFIG_NET_PKT_RX_COUNT, CONFIG_NET_BUF_RX_COUNT);
			/* Discard the packet from DM9058's memory to prevent blocking */
			dm9058_drop_packet(dev, rx_len);
			eth_stats_update_errors_rx(context->iface);
			return -ENOMEM;
		}
	}

	/* Read frame data into buffer fragments using the local implementation */
	if (!pkt->buffer) {
		LOG_WRN("%s: RX buffer allocation buffer NULL (size=%u, available pools: "
			"RX_PKT=%d, RX_BUF=%d) - discarding packet",
			dev->name, frame_len,
			CONFIG_NET_PKT_RX_COUNT, CONFIG_NET_BUF_RX_COUNT);
		/* Discard the packet from DM9058's memory to prevent blocking */
		dm9058_drop_packet(dev, rx_len);
		eth_stats_update_errors_rx(context->iface);
		return -ENOMEM;
	}

	if (local_net_pkt_read_from(pkt, dm9058_read_mem_cb, (void *)dev, frame_len)) {
		LOG_ERR("%s: Failed to write packet into fragments", dev->name);
		net_pkt_unref(pkt);
		/* Attempt to discard the rest of the packet to prevent being stuck */
		//uint8_t dummy[rx_len];
		//dm9058_read_mem(dev, dummy, rx_len);
		static uint16_t times = 0;
		LOG_ERR("dm9 pkt_read_from error times : %u", ++times);
		env_err_rst(dev);
		return -EIO;
	}

	/* Read and discard the 4-byte CRC to clear the RX buffer */
	uint8_t crc_buf[6];
	dm9058_read_mem(dev, crc_buf, (rx_len & 1) ? 5 : 4);

	/* MBNDRY_DEFAULT */
	/* Pad to even length */
	//dm9058_read_mem(dev, crc_buf, 4);
	//if (rx_len & 1) {
	//	uint8_t pad;
	//	dm9058_read_mem(dev, &pad, 1);
	//}

	dm9058_write_reg(dev, DM9058_ISR, 0x80);

	net_pkt_set_iface(pkt, context->iface);

#if defined(CONFIG_PTP_CLOCK) && defined(CONFIG_NET_GPTP)
	#if 0
	if (rx_ts_valid) {
		if (dm9058_ptp_read_time_locked(dev, &phc_ts) == 0)
			rx_phc_valid = true;
	}
	if (rx_ts_valid)
		LOG_INF("%s:  RX mem_head_timestamp %9" PRIu64 " s %9" PRIu32 " ns", dev->name, rx_ts.second, rx_ts.nanosecond);
	#endif

	if (!rx_ts_valid) {
		if (dm9058_ptp_read_time_locked(dev, &rx_ts) == 0) {
			rx_ts_valid = true;
			LOG_INF("%s 520:  RX phc_head_timestamp %9" PRIu64 " s %9" PRIu32 " ns [INSTEAD]", dev->name, rx_ts.second, rx_ts.nanosecond);
			LOG_INF("%s 521:  RX phc_head_timestamp %9" PRIu64 " s %9" PRIu32 " ns [INSTEAD]", dev->name, rx_ts.second, rx_ts.nanosecond);
		}
		else
			LOG_ERR("%s:  RX phc_head_timestamp [WANT TO INSTEAD BUT FAIL]", dev->name);
	}
	if (rx_ts_valid)
		net_pkt_set_timestamp(pkt, &rx_ts);

	#if 0
	if (rx_phc_valid) {
		LOG_INF("%s: [RX phc_read_timestamp %9" PRIu64 " s %9" PRIu32 " ns]", dev->name, phc_ts.second, phc_ts.nanosecond);
		int64_t diff_s  = (int64_t)phc_ts.second    - (int64_t)rx_ts.second;
		int64_t diff_ns = (int64_t)phc_ts.nanosecond - (int64_t)rx_ts.nanosecond;
		if (diff_ns < 0) {
			diff_s  -= 1;
			diff_ns += 1000000000LL;
		}
		LOG_INF("%s: [RX phc-rx_ts delta          %9" PRId64 " s %9" PRId64 " ns]", dev->name, diff_s, diff_ns);
	}
	#endif
#if 0
	if (rx_ts_valid) {
		net_pkt_set_timestamp(pkt, &rx_ts);
	} else if (dm9058_ptp_read_time_locked(dev, &rx_ts) == 0) {
		net_pkt_set_timestamp(pkt, &rx_ts);
		rx_ts_valid = true;
	}
#endif
#endif

	/* Feed to network stack */
	if (net_recv_data(context->iface, pkt) < 0) {
		net_pkt_unref(pkt);
		return -EIO;
	}

#if defined(CONFIG_PTP_CLOCK) && defined(CONFIG_NET_GPTP)
	if (!rx_ts_valid) {
		LOG_WRN("%s: RX packet delivered without PTP timestamp", dev->name);
	}
#endif

	// LOG_DBG("%s: RX packet len=%u", dev->name, rx_len);
	return 0;
}

/*******************************************************************************
 * RX Thread
 ******************************************************************************/

static uint8_t dm9058_link_status(const struct device *dev)
{
	// uint16_t bmsr;
	uint8_t nsr;
	struct dm9058_runtime *context = dev->data;

	// bmsr = dm9058_phy_read(dev, PHY_STATUS_REG);
	nsr = dm9058_read_reg(dev, DM9058_NSR);
	// if (bmsr == 0xffff) {
	//	LOG_ERR("%s: PHY read failed", dev->name);
	//	return;
	// }
	if (nsr == 0xff) {
		LOG_ERR("%s: NSR read failed", dev->name);
		return 0xff;
	}

	// if (bmsr & 0x01) --- PHY_STATUS_LINK = 0x0004
	if (nsr & NSR_LINKST) {
		if (context->link_up != true) {
			printk("\n");
			DM9058_DBG("\n(link_status.o=%d)\n", DM9058_ENDC_INC());
			LOG_INF("_dm9058_link_status 521: +%s: Link up", dev->name);
			context->link_up = true;
			net_eth_carrier_on(context->iface);
		}
	} else {
		if (context->link_up != false) {
			DM9058_DBG("\n(link_status.x=%d)\n", DM9058_ENDC_INC());
			LOG_INF("%s: Link down", dev->name);
			context->link_up = false;
			net_eth_carrier_off(context->iface);
		}
	}
	return nsr;
}

/**
 * @brief GPIO interrupt callback for DM9058
 * @param dev GPIO device (unused)
 * @param cb Callback structure
 * @param pins Pins that triggered the interrupt
 */
static void dm9058_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);

	struct dm9058_runtime *context = CONTAINER_OF(cb, struct dm9058_runtime, gpio_cb);

	// dm9058_interrupt_disble_irq(dev);
	k_sem_give(&context->int_sem);
}

/**
 * @brief RX thread for polling and processing incoming packets
 * @param arg1 Device structure pointer
 * @param arg2 Unused
 * @param arg3 Unused
 */
static void dm9058_rx_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	const struct device *dev = arg1;
	struct dm9058_runtime *context = dev->data;
	const struct dm9058_config *config = dev->config;
	uint32_t int_count = 0;
	uint8_t flg_print_rx_status = 0;

	while (1) {
		int loop_count = 0;
		if (cint(dev)) {
			/* Interrupt mode: wait for GPIO interrupt signal */
			int res = k_sem_take(&context->int_sem, K_MSEC(100));
			if (res == 0) {
				if (int_count % 100 == 0) {
					flg_print_rx_status = 1;
					DM9058_DBG("--------- %5d DM9058 INT.s sem_count=%u --------\n", 
					       int_count, k_sem_count_get(&context->int_sem)+1);
				}
			} else {
				/* Interrupt mode, Semaphore timeout - when no interrupt received
				 * could support update link status */
				dm9058_link_status(dev);
				continue;
			}
			dm9058_interrupt_disble_irq(dev);
		} else {
			/* Polling mode: periodic check every 10ms */
			k_sem_take(&context->int_sem, K_MSEC(config->timeout)); //polling
			/* support update link status */
			dm9058_link_status(dev);
		}

		/* Take semaphore to protect SPI access */
		k_sem_take(&context->tx_rx_sem, K_FOREVER);

		/* Process all available packets */
		while (dm9058_rx_packet(dev) == 0) {
			loop_count++;
		}

		/* Release semaphore */
		k_sem_give(&context->tx_rx_sem);
		if (flg_print_rx_status) {
			flg_print_rx_status = 0;
			DM9058_DBG("---------%5d DM9058 INT.e sem_count=%u nRX=%d--------\n", 
			       int_count, k_sem_count_get(&context->int_sem), loop_count);
		}
		int_count++;
		if (cint(dev)) {
			dm9058_interrupt_reset_for_cb_sem(dev);
		}
	}
}

/*******************************************************************************
 * Ethernet API Functions
 ******************************************************************************/

static enum ethernet_hw_caps eth_dm9058_get_capabilities(const struct device *dev)
{
	enum ethernet_hw_caps dm9058_caps;

	ARG_UNUSED(dev);

	dm9058_caps = ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE;

#if defined(CONFIG_PTP_CLOCK)
	dm9058_caps |= ETHERNET_PTP;
#endif

#if defined(CONFIG_NET_PKT_TXTIME)
	dm9058_caps |= ETHERNET_TXTIME;
#endif

#ifdef CONFIG_NET_PROMISCUOUS_MODE
	dm9058_caps |= ETHERNET_PROMISC_MODE;
#endif

#ifdef CONFIG_ETH_DM9058_MULTICAST_FILTER
	dm9058_caps |= ETHERNET_HW_FILTERING;
#endif

	return dm9058_caps;
}

#if defined(CONFIG_PTP_CLOCK)
struct dm9058_ptp_context {
	const struct device *eth_dev;
	int64_t last_rate;
	int64_t soft_base_ns;
	int64_t soft_ref_ns;
	int32_t soft_rate_ppb;
	bool use_soft_clock;
};

/* PTP register map from ESP-IDF DM9058 implementation */
#define DM9058_PTPCR                     0x60
#ifndef DM9058_PTPCW
#define DM9058_PTPCW                     0x61
#endif
#define DM9058_PTPTSM                    0x62
#define DM9058_PTPTX                     0x63
#define DM9058_PTPMMP                    0x64
#define DM9058_PTPTSO                    0x65
#define DM9058_PTPCSO                    0x66
#ifndef DM9058_PTPTS
#define DM9058_PTPTS                     0x68
#endif

#define DM9058_PTP_TCR_ENABLE            0x01
#define DM9058_PTP_TCR_RESET_INDEX       0x80
#ifndef DM9058_PTP_TCR_READ_CLOCK
#define DM9058_PTP_TCR_READ_CLOCK        0x84
#endif
#define DM9058_PTP_TCR_APPLY_SET_TIME    0x09
#define DM9058_PTP_TCR_APPLY_ADJUST_FAST 0x20
#define DM9058_PTP_TCR_APPLY_ADJUST_SLOW 0x60
#define DM9058_PTP_MAX_ADJUSTMENT        0xEFFFFFFFU

#define DM9058_PTP_TS_OFFSET_8023        0x32
#define DM9058_PTP_CS_OFFSET_8023        0x20

/* Keep this aligned with ESP-IDF v5.1 DM9058 PTP conversion */
#define V51_ADJ_FREQ_BASE_ADDEND         171.7987

#define DM9058_PTP_SANITY_MAX_DELTA_NS   (1000000000LL)

static inline int64_t dm9058_ptp_uptime_ns(void)
{
	return (int64_t)k_uptime_get() * (int64_t)NSEC_PER_MSEC;
}

static int64_t dm9058_ptp_soft_now_ns(struct dm9058_ptp_context *ctx)
{
	int64_t now = dm9058_ptp_uptime_ns();
	int64_t elapsed = now - ctx->soft_ref_ns;
	int64_t slew = (elapsed * ctx->soft_rate_ppb) / 1000000000LL;

	return ctx->soft_base_ns + elapsed + slew;
}

static void dm9058_ptp_soft_set_ns(struct dm9058_ptp_context *ctx, int64_t target_ns)
{
	ctx->soft_ref_ns = dm9058_ptp_uptime_ns();
	ctx->soft_base_ns = target_ns;
}

static inline int dm9058_ptp_try_lock(const struct device *eth_dev)
{
	struct dm9058_runtime *rt = eth_dev->data;

	if (k_sem_take(&rt->tx_rx_sem, K_MSEC(20)) != 0) {
		return -EBUSY;
	}

	return 0;
}

static inline void dm9058_ptp_unlock(const struct device *eth_dev)
{
	struct dm9058_runtime *rt = eth_dev->data;

	k_sem_give(&rt->tx_rx_sem);
}

static void dm9058_ptp_write_stream(const struct device *eth_dev, const uint8_t *src, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		dm9058_write_reg(eth_dev, DM9058_PTPTS, src[i]);
	}
}

static void dm9058_ptp_read_stream(const struct device *eth_dev, uint8_t *dst, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		dst[i] = dm9058_read_reg(eth_dev, DM9058_PTPTS);
	}
}

static int dm9058_ptp_hw_enable(const struct device *eth_dev)
{
	/* 802.1AS/gPTP over Ethernet uses L2 transport offsets */
	dm9058_write_reg(eth_dev, DM9058_PTPCR, 0x01);
	k_msleep(1);
	dm9058_write_reg(eth_dev, DM9058_PTPCR, 0x00);
	dm9058_write_reg(eth_dev, DM9058_PTPCW, DM9058_PTP_TCR_ENABLE);
	dm9058_write_reg(eth_dev, DM9058_TCR, 0x00);
	dm9058_write_reg(eth_dev, DM9058_PTPMMP, 0x12);
	dm9058_write_reg(eth_dev, DM9058_PTPTX, 0x00);
	dm9058_write_reg(eth_dev, DM9058_PTPTSO, DM9058_PTP_TS_OFFSET_8023);
	dm9058_write_reg(eth_dev, DM9058_PTPCSO, DM9058_PTP_CS_OFFSET_8023);
	dm9058_write_reg(eth_dev, DM9058_PTPTSM, 0x01);

	return 0;
}

static inline void dm9058_ptp_encode_time(const struct net_ptp_time *tm, uint8_t out[8])
{
	out[0] = (uint8_t)(tm->nanosecond);
	out[1] = (uint8_t)(tm->nanosecond >> 8);
	out[2] = (uint8_t)(tm->nanosecond >> 16);
	out[3] = (uint8_t)(tm->nanosecond >> 24);
	out[4] = (uint8_t)(tm->second);
	out[5] = (uint8_t)(tm->second >> 8);
	out[6] = (uint8_t)(tm->second >> 16);
	out[7] = (uint8_t)(tm->second >> 24);
}

static inline void dm9058_ptp_decode_time(const uint8_t in[8], struct net_ptp_time *tm)
{
	tm->nanosecond = (uint32_t)in[0] | ((uint32_t)in[1] << 8) |
			 ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
	tm->second = (uint32_t)in[4] | ((uint32_t)in[5] << 8) |
		     ((uint32_t)in[6] << 16) | ((uint32_t)in[7] << 24);
}

static int dm9058_ptp_clock_set(const struct device *dev, struct net_ptp_time *tm)
{
	struct dm9058_ptp_context *ctx = dev->data;
	uint8_t raw[8];
	int64_t target_ns;
	int ret;

	if (!ctx->eth_dev) {
		return -ENODEV;
	}

	if (tm->nanosecond >= NSEC_PER_SEC) {
		return -EINVAL;
	}

	target_ns = net_ptp_time_to_ns(tm);
	dm9058_ptp_soft_set_ns(ctx, target_ns);

	ret = dm9058_ptp_try_lock(ctx->eth_dev);
	if (ret < 0) {
		/* Keep software clock coherent even if SPI is busy. */
		ctx->use_soft_clock = true;
		return 0;
	}

	dm9058_ptp_encode_time(tm, raw);
	dm9058_write_reg(ctx->eth_dev, DM9058_PTPCR, 0x01);
	k_busy_wait(2);
	dm9058_write_reg(ctx->eth_dev, DM9058_PTPCR, 0x00);
	dm9058_write_reg(ctx->eth_dev, DM9058_PTPCW, DM9058_PTP_TCR_RESET_INDEX);
	dm9058_ptp_write_stream(ctx->eth_dev, raw, sizeof(raw));
	dm9058_write_reg(ctx->eth_dev, DM9058_PTPCW, DM9058_PTP_TCR_APPLY_SET_TIME);
	LOG_INF("%s 522:  Write phc_timestamp %9" PRIu64 " s %9" PRIu32 " ns",
		ctx->eth_dev->name, tm->second, tm->nanosecond);
	LOG_INF("%s 523:  Write phc_timestamp %9" PRIu64 " s %9" PRIu32 " ns",
		ctx->eth_dev->name, tm->second, tm->nanosecond);
	ctx->last_rate = 0;

	dm9058_ptp_unlock(ctx->eth_dev);
	ctx->use_soft_clock = false;

	return 0;
}

static int dm9058_ptp_clock_get(const struct device *dev, struct net_ptp_time *tm)
{
	struct dm9058_ptp_context *ctx = dev->data;
	struct net_ptp_time hw_tm = {0};
	int64_t soft_ns;
	int64_t hw_ns;
	int64_t diff;
	int ret;

	if (!ctx->eth_dev) {
		return -ENODEV;
	}

	soft_ns = dm9058_ptp_soft_now_ns(ctx);

	ret = dm9058_ptp_try_lock(ctx->eth_dev);
	if (ret < 0) {
		*tm = ns_to_net_ptp_time(soft_ns);
		ctx->use_soft_clock = true;
		return 0;
	}

	ret = dm9058_ptp_read_time_locked(ctx->eth_dev, &hw_tm);

	dm9058_ptp_unlock(ctx->eth_dev);
	if (ret < 0 || hw_tm.nanosecond >= NSEC_PER_SEC) {
		*tm = ns_to_net_ptp_time(soft_ns);
		ctx->use_soft_clock = true;
		return 0;
	}

	hw_ns = net_ptp_time_to_ns(&hw_tm);
	diff = hw_ns - soft_ns;
	if (diff < 0) {
		diff = -diff;
	}

	if (ctx->use_soft_clock || diff > DM9058_PTP_SANITY_MAX_DELTA_NS) {
		*tm = ns_to_net_ptp_time(soft_ns);
		ctx->use_soft_clock = true;
		return 0;
	}

	*tm = hw_tm;
	dm9058_ptp_soft_set_ns(ctx, hw_ns);
	ctx->use_soft_clock = false;

	return 0;
}

static int dm9058_ptp_clock_adjust(const struct device *dev, int increment)
{
	struct dm9058_ptp_context *ctx = dev->data;
	struct net_ptp_time now;
	int64_t sec;
	int64_t nsec;
	int64_t adjusted_ns;
	int ret;

	if (!ctx->eth_dev) {
		return -ENODEV;
	}

	ret = dm9058_ptp_clock_get(dev, &now);
	if (ret < 0) {
		return ret;
	}

	sec = (int64_t)now.second;
	nsec = (int64_t)now.nanosecond + increment;

	while (nsec >= NSEC_PER_SEC) {
		sec++;
		nsec -= NSEC_PER_SEC;
	}

	while (nsec < 0) {
		sec--;
		nsec += NSEC_PER_SEC;
	}

	if (sec < 0) {
		return -ERANGE;
	}

	now.second = (uint32_t)sec;
	now.nanosecond = (uint32_t)nsec;
	adjusted_ns = net_ptp_time_to_ns(&now);
	dm9058_ptp_soft_set_ns(ctx, adjusted_ns);

	ret = dm9058_ptp_clock_set(dev, &now);
	if (ret < 0) {
		ctx->use_soft_clock = true;
		return 0;
	}

	return ret;
}

static int dm9058_ptp_clock_rate_adjust(const struct device *dev, double ratio)
{
	struct dm9058_ptp_context *ctx = dev->data;
	int32_t adj_ppb;
	int64_t signed_addend;
	int64_t delta;
	uint32_t adjustment;
	uint8_t raw[4];
	uint8_t control_value;
	int ret;

	if (!ctx->eth_dev) {
		return -ENODEV;
	}

	adj_ppb = (int32_t)((ratio - 1.0) * 1000000000.0);
	ctx->soft_rate_ppb = adj_ppb;
	signed_addend = (int64_t)(adj_ppb * V51_ADJ_FREQ_BASE_ADDEND);
	delta = signed_addend - ctx->last_rate;

	if (delta < 0) {
		adjustment = (uint32_t)(-delta);
		control_value = DM9058_PTP_TCR_APPLY_ADJUST_SLOW;
	} else {
		adjustment = (uint32_t)delta;
		control_value = DM9058_PTP_TCR_APPLY_ADJUST_FAST;
	}

	if (adjustment > DM9058_PTP_MAX_ADJUSTMENT) {
		adjustment = DM9058_PTP_MAX_ADJUSTMENT;
	}

	raw[0] = (uint8_t)adjustment;
	raw[1] = (uint8_t)(adjustment >> 8);
	raw[2] = (uint8_t)(adjustment >> 16);
	raw[3] = (uint8_t)(adjustment >> 24);

	ret = dm9058_ptp_try_lock(ctx->eth_dev);
	if (ret < 0) {
		ctx->use_soft_clock = true;
		return 0;
	}

	dm9058_write_reg(ctx->eth_dev, DM9058_PTPCW, DM9058_PTP_TCR_RESET_INDEX);
	dm9058_ptp_write_stream(ctx->eth_dev, raw, sizeof(raw));
	dm9058_write_reg(ctx->eth_dev, DM9058_PTPCW, control_value);
	ctx->last_rate = signed_addend;

	dm9058_ptp_unlock(ctx->eth_dev);
	ctx->use_soft_clock = false;

	return 0;
}

static DEVICE_API(ptp_clock, dm9058_ptp_api) = {
	.set = dm9058_ptp_clock_set,
	.get = dm9058_ptp_clock_get,
	.adjust = dm9058_ptp_clock_adjust,
	.rate_adjust = dm9058_ptp_clock_rate_adjust,
};

static struct dm9058_ptp_context dm9058_ptp_ctx;

static int dm9058_ptp_init(const struct device *dev)
{
	struct dm9058_ptp_context *ctx = dev->data;

	ARG_UNUSED(dev);
	ctx->eth_dev = DEVICE_DT_INST_GET(0);
	ctx->last_rate = 0;
	ctx->soft_rate_ppb = 0;
	ctx->soft_ref_ns = dm9058_ptp_uptime_ns();
	ctx->soft_base_ns = ctx->soft_ref_ns;
	ctx->use_soft_clock = true;

	return 0;
}

DEVICE_DEFINE(dm9058_ptp_clock, "dm9058_ptp", dm9058_ptp_init, NULL, &dm9058_ptp_ctx, NULL,
	      POST_KERNEL, CONFIG_PTP_CLOCK_INIT_PRIORITY, &dm9058_ptp_api);

static const struct device *eth_dm9058_get_ptp_clock(const struct device *dev)
{
	ARG_UNUSED(dev);

	return DEVICE_GET(dm9058_ptp_clock);
}
#endif /* CONFIG_PTP_CLOCK */

static int dm9058_set_txtime_config(const struct device *dev,
				    const struct ethernet_txtime_param *txtime)
{
	ARG_UNUSED(dev);

	switch (txtime->type) {
	case ETHERNET_TXTIME_PARAM_TYPE_ENABLE_QUEUES:
		/*
		 * Framework hook for DM9058 timestamp/TXTIME configuration.
		 * Fill hardware register read/write here for selected queue.
		 */
		LOG_INF("%s: TXTIME queue=%d enable=%d (framework hook)",
			dev->name, txtime->queue_id, txtime->enable_txtime);
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int eth_dm9058_set_config(const struct device *dev, enum ethernet_config_type type,
				 const struct ethernet_config *config)
{
	struct dm9058_runtime *context = dev->data;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(context->mac_address, config->mac_address.addr,
		       sizeof(context->mac_address));

#if 1
		/* Set MAC address */
		dm9058_set_mac_address(dev, context->mac_address);
		LOG_INF("_dm9058_set_config: MAC, %02x:%02x:%02x:%02x:%02x:%02x",
		       context->mac_address[0], context->mac_address[1], context->mac_address[2],
		       context->mac_address[3], context->mac_address[4], context->mac_address[5]);

		/* Configure receive */
		dm9058_set_receive(dev);
#endif

		if (context->iface != NULL) {
			net_if_set_link_addr(context->iface, context->mac_address,
					     sizeof(context->mac_address), NET_LINK_ETHERNET);
		}
		return 0;

#ifdef CONFIG_NET_PROMISCUOUS_MODE
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE:
		k_sem_take(&context->tx_rx_sem, K_FOREVER);

		/* Read current RCR value */
		uint8_t rcr_value;
		rcr_value = dm9058_read_reg(dev, DM9058_RCR);

		if (config->promisc_mode) {
			/* Enable promiscuous mode */
			rcr_value |= RCR_PRMSC;
			LOG_INF("%s: Promiscuous mode enabled", dev->name);
		} else {
			/* Disable promiscuous mode */
			rcr_value &= ~RCR_PRMSC;
			LOG_INF("%s: Promiscuous mode disabled", dev->name);
		}

		/* Write updated RCR value */
		dm9058_write_reg(dev, DM9058_RCR, rcr_value);

		k_sem_give(&context->tx_rx_sem);
		return 0;
#endif

#ifdef CONFIG_ETH_DM9058_MULTICAST_FILTER
	case ETHERNET_CONFIG_TYPE_FILTER:
		/* Configure MAC Address Register (MAR) for multicast filtering */
		if (config->filter.type == ETHERNET_FILTER_TYPE_SET_MULTICAST) {
			const struct ethernet_filter_multicast *filter = &config->filter.multicast;
			uint8_t mar[8] = {0};

			k_sem_take(&context->tx_rx_sem, K_FOREVER);

			/* Calculate CRC32 hash for the multicast MAC address */
			uint32_t crc = 0xFFFFFFFF;
			for (int j = 0; j < 6; j++) {
				crc ^= filter->mac_address.addr[j];
				for (int i = 0; i < 8; i++) {
					if (crc & 1)
						crc = (crc >> 1) ^ 0xEDB88320;
					else
						crc = crc >> 1;
				}
			}

			/* Use lower 6 bits of CRC32 to determine hash table position */
			uint8_t hash_bit = crc & 0x3F;
			uint8_t mar_index = hash_bit / 8;
			uint8_t bit_index = hash_bit % 8;

			/* Read current MAR values */
			for (int i = 0; i < 8; i++) {
				mar[i] = dm9058_read_reg(dev, DM9058_MAR + i);
			}

			if (filter->enable) {
				/* Set bit in hash table */
				mar[mar_index] |= BIT(bit_index);
				LOG_DBG("%s: Added multicast filter for "
					"%02x:%02x:%02x:%02x:%02x:%02x",
					dev->name, filter->mac_address.addr[0],
					filter->mac_address.addr[1],
					filter->mac_address.addr[2],
					filter->mac_address.addr[3],
					filter->mac_address.addr[4],
					filter->mac_address.addr[5]);
			} else {
				/* Clear bit in hash table */
				mar[mar_index] &= ~BIT(bit_index);
				LOG_DBG("%s: Removed multicast filter for "
					"%02x:%02x:%02x:%02x:%02x:%02x",
					dev->name, filter->mac_address.addr[0],
					filter->mac_address.addr[1],
					filter->mac_address.addr[2],
					filter->mac_address.addr[3],
					filter->mac_address.addr[4],
					filter->mac_address.addr[5]);
			}

			/* Write updated MAR values */
			for (int i = 0; i < 8; i++) {
				dm9058_write_reg(dev, DM9058_MAR + i, mar[i]);
			}

			k_sem_give(&context->tx_rx_sem);
			return 0;
		}

		return -ENOTSUP;
#endif

	case ETHERNET_CONFIG_TYPE_TXTIME_PARAM:
		return dm9058_set_txtime_config(dev, &config->txtime_param);

	default:
		break;
	}

	LOG_DBG("%s: Unsupported configuration type %d", dev->name, type);
	return -ENOTSUP;
}

static void eth_dm9058_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct dm9058_runtime *context = dev->data;
	DM9058_ENDC_SAVE(through_c);

	net_if_set_link_addr(iface, context->mac_address, sizeof(context->mac_address),
			     NET_LINK_ETHERNET);

	if (context->iface == NULL) {
		context->iface = iface;
	}

	ethernet_init(iface);

	/* Set carrier status */
	if (context->iface_carrier_on_init) {
		net_if_carrier_on(iface);
	} else {
		net_if_carrier_off(iface);
	}

	context->iface_initialized = true;

	/* Create RX thread for packet reception */
	k_thread_create(&context->thread, context->thread_stack,
			CONFIG_ETH_DM9058_RX_THREAD_STACK_SIZE, dm9058_rx_thread, (void *)dev, NULL,
			NULL, K_PRIO_COOP(CONFIG_ETH_DM9058_RX_THREAD_PRIO),
			0, K_NO_WAIT);
	k_thread_name_set(&context->thread, "dm9058_rx");

	if (1) {
		//const struct device *dev = net_if_get_device(iface);
		const char *ifname = dev ? dev->name : "?";
		int ifindex = net_if_get_by_iface(iface);
		if (1) {
		#if 1
			char lladdr_buf[3 * 16];
			const struct net_linkaddr *lladdr = net_if_get_link_addr(iface);
			int llpos = 0;

			lladdr_buf[0] = '\0';
			if (lladdr /* && lladdr->addr*/ && lladdr->len > 0) {
				for (size_t j = 0; j < lladdr->len && j < 16; j++) {
					llpos += snprintk(lladdr_buf + llpos,
							sizeof(lladdr_buf) - llpos,
							"%s%02x",
							(j == 0) ? "" : ":",
							lladdr->addr[j]);
					if (llpos >= sizeof(lladdr_buf)) {
						break;
					}
				}
			}
			LOG_INF(" iface_init: %s (index=%d) mac=%s",
				ifname, ifindex, lladdr_buf);
		#endif
		}
	}
	DM9058_DBG("(end.e=%d)\n", DM9058_ENDC_RETRIVE(through_c));
	DM9058_DBG("iface_init.e\n");
}

static const struct ethernet_api api_funcs = {
	.iface_api.init = eth_dm9058_iface_init,
	.set_config = eth_dm9058_set_config,
	.get_capabilities = eth_dm9058_get_capabilities,
	#if defined(CONFIG_PTP_CLOCK)
	.get_ptp_clock = eth_dm9058_get_ptp_clock,
	#endif
	.send = eth_dm9058_tx,
};

void dm9058_init_debug_log(const struct device *dev)
{
	DM9058_DBG("_eth_dm9058_init: INFO: ========================================\n");
	DM9058_DBG("_eth_dm9058_init: INFO: dev->name = %s\n", dev->name);
	DM9058_DBG("_eth_dm9058_init: INFO: dev->config->spi.bus->name: %s\n",
	       ((struct dm9058_config *)dev->config)->spi.bus->name);
	DM9058_DBG("_eth_dm9058_init: INFO: dev->config->spi.config.frequency: %u MHz\n",
	       ((struct dm9058_config *)dev->config)->spi.config.frequency / 1000000);
	DM9058_DBG("_eth_dm9058_init: INFO: ========================================\n");
}

static int dm9058_config_reset_gpio(const struct device *dev)
{
	const struct dm9058_config *config = dev->config;

	if (!crst(dev)) {
		LOG_INF("_eth_dm9058_init: Skipping reset GPIO (not defined)");
		return 0;
	}

	if (!gpio_is_ready_dt(&config->reset)) {
		LOG_ERR("Reset GPIO port %s not ready", config->reset.port->name);
		return -EINVAL;
	}

	if (gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_INACTIVE)) {
		LOG_ERR("Unable to configure reset GPIO pin %u", config->reset.pin);
		return -EINVAL;
	}

	DM9058_DBG("_eth_dm9058_init: Reset GPIO configured - Port: %s, Pin: %d\n",
	       config->reset.port->name, config->reset.pin);
	return 0;
}

static int dm9058_config_interrupt_gpio(const struct device *dev)
{
	const struct dm9058_config *config = dev->config;
	struct dm9058_runtime *context = dev->data;

	if (!cint(dev)) {
		LOG_INF("_eth_dm9058_init: POLLING mode (no int-gpios defined)");
		return 0;
	}

	LOG_INF("_eth_dm9058_init: INTERRUPT mode (int-gpios defined)");

	if (!gpio_is_ready_dt(&config->interrupt)) {
		LOG_ERR("GPIO port %s not ready", config->interrupt.port->name);
		return -EINVAL;
	}

	if (gpio_pin_configure_dt(&config->interrupt, GPIO_INPUT)) {
		LOG_ERR("Unable to configure GPIO pin %u", config->interrupt.pin);
		return -EINVAL;
	}

	gpio_init_callback(&context->gpio_cb, dm9058_gpio_callback,
			   BIT(config->interrupt.pin));

	if (gpio_add_callback(config->interrupt.port, &(context->gpio_cb))) {
		return -EINVAL;
	}

	/* Use edge-to-active to respect GPIO_ACTIVE_LOW/HIGH from devicetree. */
	gpio_pin_interrupt_configure_dt(&config->interrupt, GPIO_INT_EDGE_TO_ACTIVE);
	DM9058_DBG("_eth_dm9058_init: Interrupt GPIO configured - Port: %s, Pin: %d\n",
	       config->interrupt.port->name, config->interrupt.pin);
	return 0;
}

/*******************************************************************************
 * Device Initialization
 ******************************************************************************/

/**
 * @brief Perform hardware reset using reset GPIO
 * @param dev Device structure
 */
static void dm9058_hw_reset(const struct device *dev)
{
	const struct dm9058_config *config = dev->config;

	if (!crst(dev)) {
		return;
	}

	/* Assert reset (active low) */
	gpio_pin_set_dt(&config->reset, 1);
	k_msleep(2);

	/* Deassert reset */
	gpio_pin_set_dt(&config->reset, 0);
	k_msleep(10);

	DM9058_DBG("_dm9058_hw_reset: Hardware reset complete\n");
}

/**
 * @brief Detect and verify DM9058 chip ID
 * @param dev Device structure
 * @return Chip ID on success, 0 on failure
 */
static uint16_t dm9058_detect_id(const struct device *dev)
{
	uint16_t chip_id;

	/* Try reading chip ID multiple times */
	for (int attempt = 0; attempt < 3; attempt++) {
		k_msleep(50);
		chip_id = dm9058_get_chipid(dev);
		if (chip_id == 0x9051 || chip_id == 0x9058) {
			break;
		}
	}

	/* Verify chip ID before reset */
	if (chip_id != 0x9051 && chip_id != 0x9058) {
		LOG_ERR("Invalid chip ID: 0x%04x (expected supported IDs), Re-try", chip_id);

		while (1) {
			chip_id = dm9058_get_chipid(dev);
			if (chip_id == 0x9051 || chip_id == 0x9058) {
				LOG_INF("INFO: DM9058 chip ID verified succeed: 0x%04x", chip_id);
				break;
			}
			LOG_INF(" INFO: DM9058 chip ID verified failed: 0x%04x", chip_id);
			k_msleep(1000);
		}
		return 0;
	}

	return chip_id;
}

static int dm9058_init_mac(const struct device *dev)
{
	DM9058_DBG("\n(start.s=%d) MBNDRY_DEFAULT %s\n", DM9058_ENDC_GET(),
		   MBNDRY_DEFAULT == MBNDRY_WORD ? "MBNDRY_WORD" : "NA");
		   
	/* Detect and verify chip ID */
	uint16_t chip_id = dm9058_detect_id(dev);
	if (chip_id == 0)
		return -ENODEV;

	printk("\n");
	LOG_INF("dm9058_init: +Chip ID 0x%04x", chip_id);

	/* Perform core reset */
	dm9058_core_reset(dev);

	struct dm9058_runtime *context = dev->data;
	/* Priority 1: devicetree local-mac-address (already copied into context) */
	if (dm9058_mac_is_valid(context->mac_address)) {
	LOG_INF("dm9058_init: Using DT MAC address %02x:%02x:%02x:%02x:%02x:%02x",
			context->mac_address[0], context->mac_address[1], context->mac_address[2],
			context->mac_address[3], context->mac_address[4], context->mac_address[5]);
	return 0;
	}

	/* Priority 2: try NVS */
	if (dm9058_load_mac_from_current_fit(dev, context->mac_address) == 0 &&
		dm9058_mac_is_valid(context->mac_address)) {
	LOG_INF("dm9058_init: Using CHIP MAC addr %02x:%02x:%02x:%02x:%02x:%02x",
			context->mac_address[0], context->mac_address[1], context->mac_address[2],
			context->mac_address[3], context->mac_address[4], context->mac_address[5]);
	return 0;
	}

	/* Priority 3: fallback random locally administered unicast */
	dm9058_generate_random_mac(context->mac_address);
	LOG_INF("dm9058_init: Using random MAC addr %02x:%02x:%02x:%02x:%02x:%02x",
			context->mac_address[0], context->mac_address[1], context->mac_address[2],
			context->mac_address[3], context->mac_address[4], context->mac_address[5]);
	return 0;
}

static int eth_dm9058_init(const struct device *dev)
{
	const struct dm9058_config *config = dev->config;
	struct dm9058_runtime *context = dev->data;
	int ret;

	/* Explicit banner to verify DM9058 driver selection at runtime. */
	LOG_INF("[DM9058] eth_dm9058 driver active: dev=%s", dev->name);

	/* Check SPI is ready */
	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("%s: SPI not ready", dev->name);
		return -ENODEV;
	}

	/* Print SPI configuration */
	printk("\n");
	LOG_INF("_eth_dm9058_init: +eth_dm9058_init (s8.8)");
	dm9058_init_debug_log(dev); /* Print detailed GPIO information */

	/* CS GPIO is automatically configured and controlled by SPI driver layer.
	 * No manual GPIO configuration needed when cs-gpios is set in device tree.
	 */

	/* Configure reset and interrupt GPIOs (optional) */
	ret = dm9058_config_reset_gpio(dev);
	if (ret) {
		return ret;
	}

	ret = dm9058_config_interrupt_gpio(dev);
	if (ret) {
		return ret;
	}

	/* Perform hardware reset */
	dm9058_hw_reset(dev);

	/* Decide MAC address: DT local-mac-address > NVS > random */
	if (dm9058_init_mac(dev) != 0)
		return -ENODEV;

	/* Set MAC address */
	dm9058_set_mac_address(dev, context->mac_address); // to be checked! more!

#if 1
	/* Configure multicast addresses */
	dm9058_set_multicast(dev);
#endif
	/* Configure receive */
	dm9058_set_receive(dev);

#if defined(CONFIG_PTP_CLOCK)
	/* Initialize DM9058 hardware timestamp engine for gPTP/PTP clock API. */
	ret = dm9058_ptp_hw_enable(dev);
	if (ret < 0) {
		LOG_ERR("%s: Failed to enable PTP block (%d)", dev->name, ret);
		return ret;
	}
#endif

	/* Set carrier on after successful initialization */
	context->iface_carrier_on_init = true;
	return 0;
}

/*******************************************************************************
 * Device Instantiation
 ******************************************************************************/

#define DM9058_DEFINE(inst)                                                                        \
	static struct dm9058_runtime dm9058_runtime_##inst = {                                     \
		.mac_address = DT_INST_PROP_OR(inst, local_mac_address, {0}),                              \
		.tx_rx_sem = Z_SEM_INITIALIZER((dm9058_runtime_##inst).tx_rx_sem, 1, UINT_MAX),    \
		.int_sem = Z_SEM_INITIALIZER((dm9058_runtime_##inst).int_sem, 0, UINT_MAX),        \
		.link_up = false,                                                                  \
	};                                                                                         \
                                                                                                   \
	static const struct dm9058_config dm9058_config_##inst = {                                 \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8), 0),                             \
		.interrupt = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),                       \
		.reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                         \
		.timeout_pkt = 500,                                                                  \
		.timeout = 10,                                                                     \
	};                                                                                         \
                                                                                                   \
	ETH_NET_DEVICE_DT_INST_DEFINE(inst, eth_dm9058_init, NULL, &dm9058_runtime_##inst,         \
				      &dm9058_config_##inst, CONFIG_ETH_INIT_PRIORITY, &api_funcs, \
				      NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(DM9058_DEFINE);
