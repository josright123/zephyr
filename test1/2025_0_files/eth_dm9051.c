/* DM9051 Stand-alone Ethernet Controller with SPI
 *
 * Copyright (c) 2025~2026 Davicom Semiconductor Incorporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT davicom_dm9051

#define LOG_MODULE_NAME eth_dm9051
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
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/toolchain.h>
#include <ethernet/eth_stats.h>

#include "eth_dm9051_priv.h"

/* SPI_DT_SPEC_INST_GET argument count selection:
 * Set to 1 for Zephyr versions requiring 3 args (e.g., 4.1.99: inst, operation, delay)
 * Set to 0 for Zephyr versions requiring 2 args (e.g., v4.3.0: inst, operation)
 */
#define DM9051_SPI_HAS_3_ARGS 1

#if DM9051_SPI_HAS_3_ARGS
#define DM9051_SPI_DT_GET(inst) SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8), 0)
#else
#define DM9051_SPI_DT_GET(inst) SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8))
#endif

#define ETH_DM9051_RX_THREAD_STACK_SIZE 1536 //800 (for smaller system resource device)

struct dm9051_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec interrupt;
	struct gpio_dt_spec reset;

	int32_t timeout_pkt;
	int32_t timeout;
};

struct dm9051_runtime {
	struct net_if *iface;
	struct k_sem tx_rx_sem;
	struct k_sem int_sem;

	K_KERNEL_STACK_MEMBER(thread_stack, ETH_DM9051_RX_THREAD_STACK_SIZE);
	struct k_thread thread;

	struct gpio_callback gpio_cb;

	uint8_t mac_address[6];

	bool chip_ok: 1;
	bool link_up: 1;
};

static uint8_t dm9051_read_reg(const struct device *dev, uint8_t reg);
static void dm9051_read_mem(const struct device *dev, uint8_t *buf, uint16_t len);

#ifdef CONFIG_ETH_DM9051_DEBUG_PRINTS
#define DM9051_DBG(...) printk(__VA_ARGS__)
int drvc;
#define DM9051_ENDC_INC() (drvc++)
#define DM9051_ENDC_GET() (drvc)
#define DM9051_ENDC_SAVE(sav) int sav = DM9051_ENDC_INC()
#define DM9051_ENDC_RETRIVE(sav) sav
#else
#define DM9051_DBG(...) do { } while (0)
#define DM9051_ENDC_INC() (0)
#define DM9051_ENDC_GET() (0)
#define DM9051_ENDC_SAVE(sav)
#define DM9051_ENDC_RETRIVE(sav)
#endif

/**
 * @brief Drop packet from DM9051's memory to prevent blocking
 * @param dev Device structure
 * @param rx_len Length of packet to discard (including padding)
 */
static void dm9051_drop_packet(const struct device *dev, uint16_t rx_len)
{
	/* Avoid VLA/large stack usage: discard in bounded chunks. */
	uint8_t dummy[32];
	uint16_t remaining = rx_len;

	while (remaining > 0) {
		uint16_t chunk = MIN(remaining, (uint16_t)sizeof(dummy));

		dm9051_read_mem(dev, dummy, chunk);
		remaining -= chunk;
	}
	
	/* MBNDRY_DEFAULT - Pad to even length */
	if (rx_len & 1) {
		uint8_t pad;
		dm9051_read_mem(dev, &pad, 1);
	}
}

static bool dm9051_mac_is_valid(const uint8_t mac[6])
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

static void dm9051_generate_random_mac(uint8_t mac[6])
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

int dm9051_load_mac_from_current_fit(const struct device *dev, uint8_t mac[6])
{
 for (int i = 0; i < 6; i++) {
  mac[i] = dm9051_read_reg(dev, DM9051_PAR + i);
 }
 return 0;
}

int dm9051_read_mem_cb(void *ctx, uint8_t *buf, int len)
{
	const struct device *dev = ctx;

	dm9051_read_mem(dev, buf, len);
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
	.release_version = "zephyr_dm9051_v3.1.0_v1.0",
};

/* Helper macro to check if interrupt mode is enabled based on device tree configuration */
#define crst(dev) (((const struct dm9051_config *)(dev)->config)->reset.port != NULL)
#define cint(dev) (((const struct dm9051_config *)(dev)->config)->interrupt.port != NULL)

/* DM9051 Constants */
#define DM9051_PHY     (0x40)
#define DM9051_PKT_RDY (0x01)
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
 * @brief Read single register from DM9051
 * @param dev Device structure
 * @param reg Register address
 * @return Register value
 */
static uint8_t dm9051_read_reg(const struct device *dev, uint8_t reg)
{
	const struct dm9051_config *config = dev->config;
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
 * @brief Write single register to DM9051
 * @param dev Device structure
 * @param reg Register address
 * @param val Value to write
 */
static void dm9051_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct dm9051_config *config = dev->config;
	uint8_t tx_data[2] = {reg | OPC_REG_W, val};

	struct spi_buf tx_buf = {.buf = tx_data, .len = 2};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

	int ret = spi_write_dt(&config->spi, &tx);
	if (ret < 0) {
		LOG_ERR("SPI write register failed: %d", ret);
	}
}

/**
 * @brief Read multiple bytes from DM9051 memory
 * @param dev Device structure
 * @param buf Buffer to store read data
 * @param len Number of bytes to read
 */
static void dm9051_read_mem(const struct device *dev, uint8_t *buf, uint16_t len)
{
	const struct dm9051_config *config = dev->config;
	uint8_t cmd = DM9051_MRCMD | OPC_REG_R;

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

	#if 0
	/* On some SPI bitbang implementations, long transfers may keep interrupts
	 * disabled for too long, impacting UART/shell responsiveness. Read in small
	 * chunks to bound that effect.
	 */
	const uint16_t max_chunk = 64;
	uint16_t remaining = len;
	uint16_t offset = 0;

	while (remaining > 0) {
		uint16_t chunk = MIN(remaining, max_chunk);

		const struct spi_buf tx_buf = {.buf = &cmd, .len = 1};
		const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

		const struct spi_buf rx_buf[2] = {
			{.buf = NULL, .len = 1},              /* Discard command echo */
			{.buf = buf + offset, .len = chunk},  /* Actual data */
		};
		const struct spi_buf_set rx = {.buffers = rx_buf, .count = 2};

		int ret = spi_transceive_dt(&config->spi, &tx, &rx);
		if (ret < 0) {
			LOG_ERR("SPI read memory failed: %d", ret);
			return;
		}

		offset += chunk;
		remaining -= chunk;
	}
	#endif
}

/**
 * @brief Write multiple bytes to DM9051 memory
 * @param dev Device structure
 * @param buf Buffer containing data to write
 * @param len Number of bytes to write
 */
static void dm9051_write_mem(const struct device *dev, const uint8_t *buf, uint16_t len)
{
	const struct dm9051_config *config = dev->config;
	uint8_t cmd = DM9051_MWCMD | OPC_REG_W;

	const struct spi_buf tx_buf[2] = {
		{.buf = &cmd, .len = 1},         /* Command byte */
		{.buf = (void *)buf, .len = len} /* Data bytes */
	};
	const struct spi_buf_set tx = {.buffers = tx_buf, .count = 2};

	int ret = spi_write_dt(&config->spi, &tx);
	if (ret < 0) {
		LOG_ERR("SPI write memory failed: %d", ret);
	}
	
	#if 0
	/* See dm9051_read_mem(): chunk writes to avoid very long bitbang transfers. */
	const uint16_t max_chunk = 64;
	uint16_t remaining = len;
	uint16_t offset = 0;

	while (remaining > 0) {
		uint16_t chunk = MIN(remaining, max_chunk);

		const struct spi_buf tx_buf[2] = {
			{.buf = &cmd, .len = 1},                         /* Command byte */
			{.buf = (void *)(buf + offset), .len = chunk},   /* Data bytes */
		};
		const struct spi_buf_set tx = {.buffers = tx_buf, .count = 2};

		int ret = spi_write_dt(&config->spi, &tx);
		if (ret < 0) {
			LOG_ERR("SPI write memory failed: %d", ret);
			return;
		}

		offset += chunk;
		remaining -= chunk;
	}
	#endif
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
static uint16_t dm9051_phy_read(const struct device *dev, uint16_t reg)
{
	uint16_t value;
	int timeout = 500;

	dm9051_write_reg(dev, DM9051_EPAR, DM9051_PHY | reg);
	dm9051_write_reg(dev, DM9051_EPCR, 0x0c);
	k_busy_wait(1);

	while ((dm9051_read_reg(dev, DM9051_EPCR) & 0x01) && timeout--) {
		k_busy_wait(1);
	}

	if (dm9051_read_reg(dev, DM9051_EPCR) & 0x01) {
		return 0xffff;
	}

	dm9051_write_reg(dev, DM9051_EPCR, 0x00);
	value = (dm9051_read_reg(dev, DM9051_EPDRH) << 8) | dm9051_read_reg(dev, DM9051_EPDRL);

	return value;
}
#endif

/**
 * @brief Write PHY register
 * @param dev Device structure
 * @param reg PHY register address
 * @param value Value to write
 */
static void dm9051_phy_write(const struct device *dev, uint16_t reg, uint16_t value)
{
	int timeout = 500;

	dm9051_write_reg(dev, DM9051_EPAR, DM9051_PHY | reg);
	dm9051_write_reg(dev, DM9051_EPDRL, value & 0xff);
	dm9051_write_reg(dev, DM9051_EPDRH, (value >> 8) & 0xff);
	dm9051_write_reg(dev, DM9051_EPCR, 0x0a);
	k_busy_wait(1);

	while ((dm9051_read_reg(dev, DM9051_EPCR) & 0x01) && timeout--) {
		k_busy_wait(1);
	}

	dm9051_write_reg(dev, DM9051_EPCR, 0x00);
}

void dm9051_interrupt_disble_irq(const struct device *dev)
{
	dm9051_write_reg(dev, DM9051_IMR, IMR_PAR);
}

void dm9051_isr_enab(const struct device *dev)
{
	uint8_t isrs = dm9051_read_reg(dev, DM9051_ISR);
	dm9051_write_reg(dev, DM9051_ISR, isrs);
}
void dm9051_imr_enab(const struct device *dev)
{
	dm9051_write_reg(dev, DM9051_IMR, IMR_INT_DEFAULT);
}

static void dm9051_interrupt_reset_for_cb_sem(const struct device *dev)
{
	dm9051_isr_enab(dev);
	dm9051_imr_enab(dev);
}

/*******************************************************************************
 * Core Driver Functions
 ******************************************************************************/

/**
 * @brief Perform core reset of DM9051
 * @param dev Device structure
 */
static void dm9051_core_reset(const struct device *dev)
{
	/* Power on PHY */
	dm9051_write_reg(dev, DM9051_GPR, 0x00);
	k_msleep(25);

	/* NCR reset */
	dm9051_write_reg(dev, DM9051_NCR, DM9051_NCR_RESET);
	k_msleep(5);

	/* Wait for reset completion */
	int timeout = 100;
	while ((dm9051_read_reg(dev, DM9051_NCR) & DM9051_NCR_RESET) && timeout--) {
		k_msleep(1);
	}

	/* Software defaults */
	dm9051_write_reg(dev, DM9051_MBNDRY, MBNDRY_DEFAULT);
	dm9051_write_reg(dev, DM9051_PPCR, PPCR_PAUSE_COUNT);
	dm9051_write_reg(dev, DM9051_LMCR, LMCR_MODE1);
	dm9051_write_reg(dev, DM9051_INTR, INTR_ACTIVE_LOW);

#ifdef CONFIG_ETH_DM9051_TX_CHECKSUM_OFFLOAD
	/* Enable TX checksum offload */
	dm9051_write_reg(dev, DM9051_CSCR,
			 TCSCR_IPCS_ENABLE | TCSCR_UDPCS_ENABLE | TCSCR_TCPCS_ENABLE);
#endif

#ifdef CONFIG_ETH_DM9051_RX_CHECKSUM_OFFLOAD
	/* Enable RX checksum offload */
	dm9051_write_reg(dev, DM9051_RCSSR, RCSSR_RCSEN | RCSSR_DCSE);
#endif

	LOG_DBG("%s: Core reset complete", dev->name);
}

/**
 * @brief Get chip ID
 * @param dev Device structure
 * @return Chip ID (0x9051 or 0x9058 for DM9051A)
 */
static uint16_t dm9051_get_chipid(const struct device *dev)
{
	uint16_t id;
	uint8_t pidh, pidl;

	pidh = dm9051_read_reg(dev, DM9051_PIDH);
	pidl = dm9051_read_reg(dev, DM9051_PIDL);
	id = (pidh << 8) | pidl;

	/* Print raw register values for debugging */
	// LOG_INF("DEBUG: Chip ID registers - PIDH: 0x%02x, PIDL: 0x%02x, Combined: 0x%04x", pidh,
	//	pidl, id);

	/* DM9051 returns 0x9000, normalize to 0x9051 */
	if (id == 0x9000) {
		id = 0x9051;
		LOG_INF("DEBUG: Normalized chip ID from 0x9000 to 0x9051");
	}

	return id;
}

/**
 * @brief Set MAC address
 * @param dev Device structure
 * @param mac MAC address array (6 bytes)
 */
static void dm9051_set_mac_address(const struct device *dev, const uint8_t *mac)
{
	for (int i = 0; i < 6; i++) {
		dm9051_write_reg(dev, DM9051_PAR + i, mac[i]);
	}
}

/**
 * @brief Configure multicast address registers
 * @param dev Device structure
 */
static void dm9051_set_multicast(const struct device *dev)
{
	for (int i = 0; i < 8; i++) {
		dm9051_write_reg(dev, DM9051_MAR + i, (i == 7) ? 0x80 : 0x00);
	}
}

/**
 * @brief Configure receive settings
 * @param dev Device structure
 */
static void dm9051_set_receive(const struct device *dev)
{
#if 0
	/* Configure multicast addresses */
	dm9051_set_multicast(dev);
#endif
	/* Configure flow control */
	dm9051_write_reg(dev, DM9051_FCR, FCR_DEFAULT);
	dm9051_phy_write(dev, PHY_ADV_REG, 0x0400 | 0x01e1);

	/* Configure interrupts based on device tree configuration */
	if (cint(dev)) {
		/* Interrupt mode enabled via int-gpios in device tree */
		dm9051_write_reg(dev, DM9051_IMR, IMR_INT_DEFAULT);
	} else {
		/* Polling mode (no int-gpios defined) */
		dm9051_write_reg(dev, DM9051_IMR, IMR_POL_DEFAULT);
	}

	/* Enable receiver */
	dm9051_write_reg(dev, DM9051_RCR, RCR_DEFAULT | RCR_RXEN);

	LOG_DBG("%s: Receive configured (%s mode)", dev->name, cint(dev) ? "INTERRUPT" : "POLLING");
}

/*******************************************************************************
 * Packet Transmission
 ******************************************************************************/

/**
 * @brief Transmit packet
 * @param dev Device structure
 * @param pkt Network packet
 * @return 0 on success, negative errno on failure
 */
static int eth_dm9051_tx(const struct device *dev, struct net_pkt *pkt)
{
	struct dm9051_runtime *context = dev->data;
	uint16_t len = net_pkt_get_len(pkt);
	struct net_buf *frag;
	int timeout = 500;

	LOG_DBG("%s: TX packet len=%u", dev->name, len);

	k_sem_take(&context->tx_rx_sem, K_FOREVER);

	/* tx pad default_boundary */
	//uint16_t pad_len = (MBNDRY_DEFAULT == MBNDRY_WORD) && (len & 1) ? len + 1 : len;

	/* Set packet length */
	dm9051_write_reg(dev, DM9051_TXPLL, len & 0xff);
	dm9051_write_reg(dev, DM9051_TXPLH, (len >> 8) & 0xff);

	/* Write packet data */
	//uint16_t pad = 0;
	for (frag = pkt->frags; frag; frag = frag->frags) {
		//if ((MBNDRY_DEFAULT == MBNDRY_WORD) && !frag->frags && (frag->len & 1))
		//	pad = 1;
		dm9051_write_mem(dev, frag->data, frag->len); // + pad
	}

	/* MBNDRY_DEFAULT */
	/* Pad to even length */
	if (len & 1) {
		uint8_t pad = 0x00;
		dm9051_write_mem(dev, &pad, 1);
	}

	/* Trigger transmission */
	dm9051_write_reg(dev, DM9051_TCR, TCR_TXREQ);

	/* Wait for completion with timeout */
	while (timeout--) {
		if (!(dm9051_read_reg(dev, DM9051_TCR) & TCR_TXREQ)) {
			break;
		}
		k_busy_wait(1);
	}

	k_sem_give(&context->tx_rx_sem);

	if (timeout == 0) {
		LOG_ERR("%s: TX timeout", dev->name);
		return -ETIMEDOUT;
	}

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
  dm9051_core_reset(dev); //cspi_core_reset();
  dm9051_set_receive(dev); //cspi_core_start1();
}

/**
 * @brief Check if RX packet is ready
 * @param dev Device structure
 * @return true if packet ready, false otherwise
 */
static bool dm9051_rx_ready(const struct device *dev)
{
	uint8_t rxbyte;

	/* Read RX byte twice (dummy read first) */
	rxbyte = dm9051_read_reg(dev, DM9051_MRCMDX);
	rxbyte = dm9051_read_reg(dev, DM9051_MRCMDX);

	return (rxbyte & 0x01) == DM9051_PKT_RDY;
}

/**
 * @brief Receive packet
 * @param dev Device structure
 * @return 0 on success, negative errno on failure
 */
static int dm9051_rx_packet(const struct device *dev)
{
	const struct dm9051_config *config = dev->config;
	struct dm9051_runtime *context = dev->data;
	uint8_t header[4];
	uint16_t rx_len;
	uint8_t rx_status;
	struct net_pkt *pkt;

	k_sem_take(&context->tx_rx_sem, K_FOREVER);
	if (!dm9051_rx_ready(dev)) {
		k_sem_give(&context->tx_rx_sem);
		return 1; // 0;
	}

	/* Read packet header */
	dm9051_read_mem(dev, header, 4);
	dm9051_write_reg(dev, DM9051_ISR, 0x80);

	rx_status = header[1];
	rx_len = header[2] | (header[3] << 8);

	/* Validate packet */
	if (rx_status & RSR_ERR_BITS) {
		LOG_ERR("%s: RX error status=0x%02x", dev->name, rx_status);
		env_err_rst(dev);
		k_sem_give(&context->tx_rx_sem);
		return -EIO;
	}

	if (rx_len > NET_ETH_MTU + sizeof(struct net_eth_hdr) + 4 || rx_len < 4) {
		LOG_ERR("%s: RX length error len=%u", dev->name, rx_len);
		env_err_rst(dev);
		k_sem_give(&context->tx_rx_sem);
		return -EINVAL;
	}

	/* rx_len default_boundary */
	//if (MBNDRY_DEFAULT == MBNDRY_WORD)
	//	rx_len = ((rx_len + 1) >> 1) << 1; 


	/* rx_len from chip includes 4-byte CRC, but net_pkt is for frame data only */
	uint16_t frame_len = rx_len - 4;

	/* Allocate packet buffer for the frame with timeout retry strategy
	 * First attempt: use configured timeout
	 * If that fails, try with K_NO_WAIT in case buffers become available
	 */
	pkt = net_pkt_rx_alloc_with_buffer(context->iface, frame_len, AF_UNSPEC, 0,
					   K_MSEC(config->timeout_pkt));
	if (!pkt) {
		/* Retry once without blocking - buffers may have been freed by RX thread */
		pkt = net_pkt_rx_alloc_with_buffer(context->iface, frame_len, AF_UNSPEC, 0,
						   K_NO_WAIT);
		if (!pkt) {
			LOG_WRN("%s: RX buffer allocation failed (size=%u, available pools: "
				"RX_PKT=%d, RX_BUF=%d) - discarding packet",
				dev->name, frame_len,
				CONFIG_NET_PKT_RX_COUNT, CONFIG_NET_BUF_RX_COUNT);
			/* Discard the packet from DM9051's memory to prevent blocking */
			dm9051_drop_packet(dev, rx_len);
			eth_stats_update_errors_rx(context->iface);
			k_sem_give(&context->tx_rx_sem);
			return -ENOMEM;
		}
	}

	/* Read frame data into buffer fragments using the local implementation */
	if (!pkt->buffer) {
		LOG_WRN("%s: RX buffer allocation buffer NULL (size=%u, available pools: "
			"RX_PKT=%d, RX_BUF=%d) - discarding packet",
			dev->name, frame_len,
			CONFIG_NET_PKT_RX_COUNT, CONFIG_NET_BUF_RX_COUNT);
		/* Discard the packet from DM9051's memory to prevent blocking */
		dm9051_drop_packet(dev, rx_len);
		eth_stats_update_errors_rx(context->iface);
		k_sem_give(&context->tx_rx_sem);
		return -ENOMEM;
	}

	if (local_net_pkt_read_from(pkt, dm9051_read_mem_cb, (void *)dev, frame_len)) {
		LOG_ERR("%s: Failed to write packet into fragments", dev->name);
		net_pkt_unref(pkt);
		/* Attempt to discard the rest of the packet to prevent being stuck */
		//uint8_t dummy[rx_len];
		//dm9051_read_mem(dev, dummy, rx_len);
		static uint16_t times = 0;
		LOG_ERR("dm9 pkt_read_from error times : %u", ++times);
		env_err_rst(dev);
		k_sem_give(&context->tx_rx_sem);
		return -EIO;
	}

	/* Read and discard the 4-byte CRC to clear the RX buffer */
	uint8_t crc_buf[4];
	dm9051_read_mem(dev, crc_buf, 4);

	/* MBNDRY_DEFAULT */
	/* Pad to even length */
	if (rx_len & 1) {
		uint8_t pad;
		dm9051_read_mem(dev, &pad, 1);
	}

	dm9051_write_reg(dev, DM9051_ISR, 0x80);
	k_sem_give(&context->tx_rx_sem);

	net_pkt_set_iface(pkt, context->iface);

	/* Feed to network stack - OUTSIDE the SPI semaphore to avoid deadlocks */
	if (net_recv_data(context->iface, pkt) < 0) {
		net_pkt_unref(pkt);
		return -EIO;
	}

	// LOG_DBG("%s: RX packet len=%u", dev->name, rx_len);
	return 0;
}

/*******************************************************************************
 * RX Thread
 ******************************************************************************/

static uint8_t dm9051_link_status(const struct device *dev)
{
	uint8_t nsr;
	struct dm9051_runtime *context = dev->data;
	bool carrier_on = false;
	bool carrier_off = false;

	k_sem_take(&context->tx_rx_sem, K_FOREVER);
	nsr = dm9051_read_reg(dev, DM9051_NSR);
	if (nsr == 0xff) {
		LOG_ERR("%s: NSR read failed", dev->name);
		k_sem_give(&context->tx_rx_sem);
		return 0xff;
	}

	/* Link change notifications require a valid interface. */
	if (context->iface == NULL) {
		k_sem_give(&context->tx_rx_sem);
		return nsr;
	}

	if (nsr & NSR_LINKST) {
		if (context->link_up != true) {
			context->link_up = true;
			carrier_on = true;
		}
	} else {
		if (context->link_up != false) {
			context->link_up = false;
			carrier_off = true;
		}
	}
	k_sem_give(&context->tx_rx_sem);

	if (carrier_on) {
		printk("_dm9051_link_status: +%s: Link up\n", dev->name);
		net_eth_carrier_on(context->iface);
	} else if (carrier_off) {
		printk("%s: Link down\n", dev->name);
		net_eth_carrier_off(context->iface);
	}

	return nsr;
}

/**
 * @brief GPIO interrupt callback for DM9051
 * @param dev GPIO device (unused)
 * @param cb Callback structure
 * @param pins Pins that triggered the interrupt
 */
static void dm9051_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);

	struct dm9051_runtime *context = CONTAINER_OF(cb, struct dm9051_runtime, gpio_cb);

	// dm9051_interrupt_disble_irq(dev);
	k_sem_give(&context->int_sem);
}

/**
 * @brief RX thread for polling and processing incoming packets
 * @param arg1 Device structure pointer
 * @param arg2 Unused
 * @param arg3 Unused
 */
static void dm9051_rx_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	const struct device *dev = arg1;
	struct dm9051_runtime *context = dev->data;
	/*const struct dm9051_config *config = dev->config;*/
	uint32_t int_count = 0;
	uint8_t flg_print_rx_status = 0;

	
	/* Guard time to let system stabilize and finish booting */
	k_msleep(200);
	while (1) {
		/* Limit how many frames we process per wake-up so we don't
		 * starve other threads (e.g. shell/console) on busy networks.
		 */
		const int rx_burst_max = 8;
		int rx_burst = 0;
		int loop_count = 0;
		if (cint(dev)) {
			/* Interrupt mode: wait for GPIO interrupt signal */
			int res = k_sem_take(&context->int_sem, K_MSEC(100));
			if (res == 0) {
				if (int_count % 100 == 0) {
					flg_print_rx_status = 1;
					DM9051_DBG("--------- %5d DM9051 INT.s sem_count=%u --------\n", 
					       int_count, k_sem_count_get(&context->int_sem)+1);
				}
				
				k_sem_take(&context->tx_rx_sem, K_FOREVER);
				dm9051_interrupt_disble_irq(dev);
				k_sem_give(&context->tx_rx_sem);
			} else {
				/* Interrupt mode, Semaphore timeout - when no interrupt received
				 * could support update link status */
				dm9051_link_status(dev);
				continue;
			}
		} else {
			/* Polling mode: periodic check every 10ms */
			/*k_sem_take(&context->int_sem, K_MSEC(config->timeout));*/ //polling
			k_msleep(10);
		}

		/* Process all available packets */
		while (dm9051_rx_packet(dev) == 0) {
			loop_count++;
			if (++rx_burst >= rx_burst_max) {
				break;
			}
		}

		/* support update link status */
		dm9051_link_status(dev);

		/* If we hit the burst limit, or processed packets, yield so other threads can run. */
		if (rx_burst >= rx_burst_max || loop_count > 0) {
			k_yield();
			k_msleep(1);
		}

		if (flg_print_rx_status) {
			flg_print_rx_status = 0;
			DM9051_DBG("---------%5d DM9051 INT.e sem_count=%u nRX=%d--------\n", 
			       int_count, k_sem_count_get(&context->int_sem), loop_count);
		}
		
		int_count++;
		if (cint(dev)) {
			k_sem_take(&context->tx_rx_sem, K_FOREVER);
			dm9051_interrupt_reset_for_cb_sem(dev);
			k_sem_give(&context->tx_rx_sem);
		}
	}
}

/*******************************************************************************
 * Ethernet API Functions
 ******************************************************************************/

static enum ethernet_hw_caps eth_dm9051_get_capabilities(const struct device *dev)
{
	enum ethernet_hw_caps dm9051_caps;

	ARG_UNUSED(dev);

	dm9051_caps = ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE;

#ifdef CONFIG_NET_PROMISCUOUS_MODE
	dm9051_caps |= ETHERNET_PROMISC_MODE;
#endif

#ifdef CONFIG_ETH_DM9051_MULTICAST_FILTER
	dm9051_caps |= ETHERNET_HW_FILTERING;
#endif

	return dm9051_caps;
}

static int eth_dm9051_set_config(const struct device *dev, enum ethernet_config_type type,
				 const struct ethernet_config *config)
{
	struct dm9051_runtime *context = dev->data;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(context->mac_address, config->mac_address.addr,
		       sizeof(context->mac_address));

		k_sem_take(&context->tx_rx_sem, K_FOREVER);
		/* Set MAC address */
		dm9051_set_mac_address(dev, context->mac_address);
		LOG_INF("_dm9051_set_config: MAC, %02x:%02x:%02x:%02x:%02x:%02x",
		       context->mac_address[0], context->mac_address[1], context->mac_address[2],
		       context->mac_address[3], context->mac_address[4], context->mac_address[5]);

		/* Configure receive */
		dm9051_set_receive(dev);
		k_sem_give(&context->tx_rx_sem);

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
		rcr_value = dm9051_read_reg(dev, DM9051_RCR);

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
		dm9051_write_reg(dev, DM9051_RCR, rcr_value);

		k_sem_give(&context->tx_rx_sem);
		return 0;
#endif

#ifdef CONFIG_ETH_DM9051_MULTICAST_FILTER
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
				mar[i] = dm9051_read_reg(dev, DM9051_MAR + i);
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
				dm9051_write_reg(dev, DM9051_MAR + i, mar[i]);
			}

			k_sem_give(&context->tx_rx_sem);
			return 0;
		}

		return -ENOTSUP;
#endif

	default:
		break;
	}

	LOG_DBG("%s: Unsupported configuration type %d", dev->name, type);
	return -ENOTSUP;
}

static void eth_dm9051_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct dm9051_runtime *context = dev->data;
	DM9051_ENDC_SAVE(through_c);

	net_if_set_link_addr(iface, context->mac_address, sizeof(context->mac_address),
			     NET_LINK_ETHERNET);

	if (context->iface == NULL) {
		context->iface = iface;
	}

	ethernet_init(iface);

	/* Set carrier status */
	if (context->link_up)
	{
		net_if_carrier_on(iface);
	} else {
		net_if_carrier_off(iface);
	}

	/* Create RX thread for packet reception */
	k_thread_create(&context->thread, context->thread_stack,
			K_KERNEL_STACK_SIZEOF(context->thread_stack), dm9051_rx_thread, (void *)dev, NULL,
			NULL, K_PRIO_PREEMPT(CONFIG_ETH_DM9051_RX_THREAD_PRIO), /* Higher priority for network RX */
			0, K_NO_WAIT);
	k_thread_name_set(&context->thread, "dm9051_rx");

	DM9051_DBG("(iface_init.e=%d) %s: struct runtime link_up = %s\n", DM9051_ENDC_RETRIVE(through_c), 
		dev->name, context->link_up ? "true" : "false");
}

static const struct ethernet_api api_funcs = {
	.iface_api.init = eth_dm9051_iface_init,
	.set_config = eth_dm9051_set_config,
	.get_capabilities = eth_dm9051_get_capabilities,
	.send = eth_dm9051_tx,
};

void dm9051_init_title_log(char *head)
{
	printk("\n");
	LOG_INF("_eth_dm9051_init: +eth_dm9051_init %s", head);
}

void dm9051_init_debug_log(const struct device *dev)
{
	DM9051_DBG("(dm9051_init_debug_log) ========================================\n");
	DM9051_DBG("(dm9051_init_debug_log) dev->name = %s\n", dev->name);
	DM9051_DBG("(dm9051_init_debug_log) dev->config->spi.bus->name: %s\n",
	       ((struct dm9051_config *)dev->config)->spi.bus->name);
	DM9051_DBG("(dm9051_init_debug_log) dev->config->spi.config.frequency: %u MHz\n",
	       ((struct dm9051_config *)dev->config)->spi.config.frequency / 1000000);
	DM9051_DBG("(dm9051_init_debug_log) ========================================\n");
}

int dm9051_init_chip_log(char *head, const struct device *dev, uint16_t chip_id)
{
	struct dm9051_runtime *context = dev->data;

	printk("\n");
	/*LOG_INF("_dm9051_mac: +ChipID %04x, Using %s %02x:%02x:%02x:%02x:%02x:%02x",*/
	LOG_INF("_dm9051_mac: +ChipID %04x, Using %s %02x%02x%02x%02x%02x%02x", chip_id, head,
			context->mac_address[0], context->mac_address[1], context->mac_address[2],
			context->mac_address[3], context->mac_address[4], context->mac_address[5]);
	return 0;
}

static int dm9051_config_reset_gpio(const struct device *dev)
{
	const struct dm9051_config *config = dev->config;

	if (!crst(dev)) {
		LOG_INF("_eth_dm9051_init: Skipping reset GPIO (not defined)");
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

	DM9051_DBG("_eth_dm9051_init: Reset GPIO configured - Port: %s, Pin: %d\n",
	       config->reset.port->name, config->reset.pin);
	return 0;
}

static int dm9051_config_interrupt_gpio(const struct device *dev)
{
	const struct dm9051_config *config = dev->config;
	struct dm9051_runtime *context = dev->data;

	if (!cint(dev)) {
		LOG_INF("_eth_dm9051_init: POLLING mode (no int-gpios defined)");
		return 0;
	}

	LOG_INF("_eth_dm9051_init: INTERRUPT mode (int-gpios defined)");

	if (!gpio_is_ready_dt(&config->interrupt)) {
		LOG_ERR("GPIO port %s not ready", config->interrupt.port->name);
		return -EINVAL;
	}

	if (gpio_pin_configure_dt(&config->interrupt, GPIO_INPUT)) {
		LOG_ERR("Unable to configure GPIO pin %u", config->interrupt.pin);
		return -EINVAL;
	}

	gpio_init_callback(&context->gpio_cb, dm9051_gpio_callback,
			   BIT(config->interrupt.pin));

	if (gpio_add_callback(config->interrupt.port, &(context->gpio_cb))) {
		return -EINVAL;
	}

	/* Use edge-to-active to respect GPIO_ACTIVE_LOW/HIGH from devicetree. */
	gpio_pin_interrupt_configure_dt(&config->interrupt, GPIO_INT_EDGE_TO_ACTIVE);
	DM9051_DBG("_eth_dm9051_init: Interrupt GPIO configured - Port: %s, Pin: %d\n",
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
static void dm9051_hw_reset(const struct device *dev)
{
	const struct dm9051_config *config = dev->config;

	if (!crst(dev)) {
		return;
	}

	/* Assert reset (active low) */
	gpio_pin_set_dt(&config->reset, 1);
	k_msleep(2);

	/* Deassert reset */
	gpio_pin_set_dt(&config->reset, 0);
	k_msleep(10);

	DM9051_DBG("_dm9051_hw_reset: Hardware reset complete\n");
}

/**
 * @brief Detect and verify DM9051 chip ID
 * @param dev Device structure
 * @return Chip ID on success, 0 on failure
 */
static uint16_t dm9051_detect_id(const struct device *dev)
{
	uint16_t chip_id;

	/* Try reading chip ID multiple times */
	for (int attempt = 0; attempt < 3; attempt++) {
		k_msleep(50);
		chip_id = dm9051_get_chipid(dev);
		if (chip_id == 0x9051 || chip_id == 0x9058) {
			break;
		}
	}

	/* Verify chip ID before reset. Never block boot forever: retry for a
	 * bounded amount of time and return failure if the device is not responding.
	 */
	if (chip_id != 0x9051 && chip_id != 0x9058) {
		LOG_ERR("Invalid chip ID: 0x%04x (expected 0x9051 or 0x9058)", chip_id);

		for (int attempt = 0; attempt < CONFIG_ETH_DM9051_ID_VERIFY_RETRY_COUNT; attempt++) {
			chip_id = dm9051_get_chipid(dev);
			if (chip_id == 0x9051 || chip_id == 0x9058) {
				LOG_INF("DM9051 chip ID verified: 0x%04x", chip_id);
				return chip_id;
			}
			k_msleep(CONFIG_ETH_DM9051_ID_VERIFY_RETRY_DELAY_MS);
		}

		LOG_ERR("DM9051 chip ID verify failed after %d retries",
			CONFIG_ETH_DM9051_ID_VERIFY_RETRY_COUNT);
		return 0;
	}

	return chip_id;
}

static int dm9051_init_mac(const struct device *dev)
{
	DM9051_ENDC_SAVE(through_c);
	DM9051_DBG("(start.s=%d) MBNDRY_DEFAULT %s\n", through_c,
		   MBNDRY_DEFAULT == MBNDRY_WORD ? "MBNDRY_WORD" : "NA");
		   
	/* Detect and verify chip ID */
	uint16_t chip_id = dm9051_detect_id(dev);
	if (chip_id == 0)
		return -ENODEV;

	/* Perform core reset */
	dm9051_core_reset(dev);

	struct dm9051_runtime *context = dev->data;
	/* Priority 1: devicetree local-mac-address (already copied into context) */
	if (dm9051_mac_is_valid(context->mac_address)) {
		return dm9051_init_chip_log("DT MAC address", dev, chip_id);
	}

	/* Priority 2: try NVS */
	if (dm9051_load_mac_from_current_fit(dev, context->mac_address) == 0 &&
		dm9051_mac_is_valid(context->mac_address)) {
		return dm9051_init_chip_log("CHIP MAC addr", dev, chip_id);
	}

	/* Priority 3: fallback random locally administered unicast */
	dm9051_generate_random_mac(context->mac_address);
	return dm9051_init_chip_log("random MAC addr", dev, chip_id);
}

static int eth_dm9051_init(const struct device *dev)
{
	const struct dm9051_config *config = dev->config;
	struct dm9051_runtime *context = dev->data;
	int ret;

	/* Check SPI is ready */
	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("%s: SPI not ready", dev->name);
		return -ENODEV;
	}

	dm9051_init_title_log("(s8.8)");

	/* CS GPIO is automatically configured and controlled by SPI driver layer.
	 * No manual GPIO configuration needed when cs-gpios is set in device tree.
	 */

	/* Configure reset and interrupt GPIOs (optional) */
	ret = dm9051_config_reset_gpio(dev);
	if (ret) {
		return ret;
	}

	ret = dm9051_config_interrupt_gpio(dev);
	if (ret) {
		return ret;
	}

	dm9051_init_debug_log(dev); /* Print detailed GPIO information */

	/* Perform hardware reset */
	dm9051_hw_reset(dev);

	/* Decide MAC address: DT local-mac-address > NVS > random */
	if (dm9051_init_mac(dev) != 0)
		return -ENODEV;

	/* Set MAC address */
	dm9051_set_mac_address(dev, context->mac_address); // to be checked! more!

#if 1
	/* Configure multicast addresses */
	dm9051_set_multicast(dev);
#endif
	/* Configure receive */
	dm9051_set_receive(dev);
	return 0;
}

/*******************************************************************************
 * Device Instantiation
 ******************************************************************************/

#define DM9051_DEFINE(inst)                                                                        \
	static struct dm9051_runtime dm9051_runtime_##inst = {                                     \
		.mac_address = DT_INST_PROP(inst, local_mac_address),                              \
		/* Binary semaphores: cap count to 1 to avoid backlog/starvation. */                \
		.tx_rx_sem = Z_SEM_INITIALIZER((dm9051_runtime_##inst).tx_rx_sem, 1, 1),           \
		.int_sem = Z_SEM_INITIALIZER((dm9051_runtime_##inst).int_sem, 0, 1),               \
		.link_up = false,                                                                  \
	};                                                                                         \
                                                                                                   \
	static const struct dm9051_config dm9051_config_##inst = {                                 \
		.spi = DM9051_SPI_DT_GET(inst),                                                    \
		.interrupt = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),                       \
		.reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                         \
		.timeout_pkt = 500,                                                                \
		.timeout = 10,                                                                     \
	};                                                                                         \
                                                                                                   \
	ETH_NET_DEVICE_DT_INST_DEFINE(inst, eth_dm9051_init, NULL, &dm9051_runtime_##inst,         \
				      &dm9051_config_##inst, CONFIG_ETH_INIT_PRIORITY, &api_funcs, \
				      NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(DM9051_DEFINE);
