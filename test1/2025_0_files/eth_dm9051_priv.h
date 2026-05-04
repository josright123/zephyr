/* ENC28J60 Stand-alone Ethernet Controller with SPI
 *
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#ifndef _DM9051_
#define _DM9051_

#define MAC_ADDR_LENGTH (6)
typedef uint8_t mac_t[MAC_ADDR_LENGTH];

/* internal */
#define DM9051_NCR   (0x00)
#define DM9051_NSR   (0x01)
#define DM9051_TCR   (0x02)
#define DM9051_TSR1  (0x03)
#define DM9051_TSR2  (0x04)
#define DM9051_RCR   (0x05)
#define DM9051_RSR   (0x06)
#define DM9051_ROCR  (0x07)
#define DM9051_BPTR  (0x08)
#define DM9051_FCTR  (0x09)
#define DM9051_FCR   (0x0A)
#define DM9051_EPCR  (0x0B)
#define DM9051_EPAR  (0x0C)
#define DM9051_EPDRL (0x0D)
#define DM9051_EPDRH (0x0E)
#define DM9051_WCR   (0x0F)

#define DM9051_PAR (0x10)
#define DM9051_MAR (0x16)

#define DM9051_GPCR  (0x1e)
#define DM9051_GPR   (0x1f)
#define DM9051_TRPAL (0x22)
#define DM9051_TRPAH (0x23)
#define DM9051_RWPAL (0x24)
#define DM9051_RWPAH (0x25)

#define DM9051_VIDL (0x28)
#define DM9051_VIDH (0x29)
#define DM9051_PIDL (0x2A)
#define DM9051_PIDH (0x2B)

#define DM9051_CHIPR (0x2C)
#define DM9051_TCR2  (0x2D)
#define DM9051_OTCR  (0x2E)
#define DM9051_SMCR  (0x2F)

#define DM9051_ATCR  (0x30) /* early transmit control/status register */
#define DM9051_CSCR  (0x31) /* check sum control register */
#define DM9051_RCSSR (0x32) /* receive check sum status register */

#define DM9051_PBCR   (0x38)
#define DM9051_INTR   (0x39)
#define DM9051_TXFSSR (0x3B)
#define DM9051_PPCR   (0x3D)
#define DM9051_IPCOCR (0x54)
#define DM9051_MPCR   (0x55)
#define DM9051_LMCR   (0x57)
#define DM9051_MBNDRY (0x5E)
#define DM9051_MRCMDX (0x70)
// #define DM9051_MRCMD              (0x72)
#define DM9051_MRRL   (0x74)
#define DM9051_MRRH   (0x75)
#define DM9051_MWCMDX (0x76)
// #define DM9051_MWCMD              (0x78)
#define DM9051_MWRL   (0x7A)
#define DM9051_MWRH   (0x7B)
#define DM9051_TXPLL  (0x7C)
#define DM9051_TXPLH  (0x7D)
#define DM9051_ISR    (0x7E)
#define DM9051_IMR    (0x7F)

/* Parameters for read operation used in _spi_data_read()/_spi_mem2x_read()/_spi_mem_read() */
// #define OPC_REG_W                 0x80             // Register Write
// #define OPC_REG_R                 0x00             // Register Read

#define CHIPR_DM9051A (0x19)
#define CHIPR_DM9051B (0x1B)

#define DM9051_NCR_RESET (0x01)
#define DM9051_IMR_OFF   (0x80)
#define DM9051_TCR2_SET  (0x90) /* set one packet */
#define DM9051_RCR_SET   (0x31)
#define DM9051_BPTR_SET  (0x37)
#define DM9051_FCTR_SET  (0x38)
#define DM9051_FCR_SET   (0x28)
// #define DM9051_TCR_SET        (0x01)

#define NCR_EXT_PHY (1 << 7)
#define NCR_WAKEEN  (1 << 6)
#define NCR_FCOL    (1 << 4)
#define NCR_FDX     (1 << 3)
#define NCR_LBK     (3 << 1)
#define NCR_RST     (1 << 0)
#define NCR_DEFAULT (0x0) /* Disable Wakeup */

#define NSR_SPEED      (1 << 7)
#define NSR_LINKST     (1 << 6)
#define NSR_WAKEST     (1 << 5)
#define NSR_TX2END     (1 << 3)
#define NSR_TX1END     (1 << 2)
#define NSR_RXOV       (1 << 1)
#define NSR_RXRDY      (1 << 0)
#define NSR_CLR_STATUS (NSR_WAKEST | NSR_TX2END | NSR_TX1END)

#define TCR_RSV_BIT7 (1 << 7) /* _15888_ */
#define TCR_TJDIS    (1 << 6) /* _15888_ too */
#define TCR_EXCECM   (1 << 5)
#define TCR_PAD_DIS2 (1 << 4)
#define TCR_CRC_DIS2 (1 << 3)
#define TCR_PAD_DIS1 (1 << 2)
#define TCR_CRC_DIS1 (1 << 1)
#define TCR_TXREQ    (1 << 0) /* Start TX */
#define TCR_DEFAULT  (0x0)

#define TSR_TJTO (1 << 7)
#define TSR_LC   (1 << 6)
#define TSR_NC   (1 << 5)
#define TSR_LCOL (1 << 4)
#define TSR_COL  (1 << 3)
#define TSR_EC   (1 << 2)

#define RCR_WTDIS    (1 << 6)
#define RCR_DIS_LONG (1 << 5)
#define RCR_DIS_CRC  (1 << 4)
#define RCR_ALL      (1 << 3)
#define RCR_RUNT     (1 << 2)
#define RCR_PRMSC    (1 << 1)
#define RCR_RXEN     (1 << 0)
// #ifdef FORCE_RCR_ALL
// #define RCR_DEFAULT           (RCR_DIS_LONG | RCR_DIS_CRC | RCR_ALL)
// #else
// #endif
#define RCR_DEFAULT  (RCR_DIS_LONG | RCR_DIS_CRC)

#define RSR_RF   (1 << 7)
#define RSR_MF   (1 << 6)
#define RSR_LCS  (1 << 5)
#define RSR_RWTO (1 << 4)
#define RSR_PLE  (1 << 3)
#define RSR_AE   (1 << 2)
#define RSR_CE   (1 << 1)
#define RSR_FOE  (1 << 0)

#define INTR_ACTIVE_LOW (1 << 0)

/* 0x0A */
#define FCR_TXPEN (1 << 5)
#define FCR_BKPA  (1 << 4)
#define FCR_BKPM  (1 << 3)
#define FCR_FLCE  (1 << 0)

/* 0x30 */
#define ATCR_TX_MODE2 (1 << 4)

/* 0x31 */
// #define _DM9051_CSCR         (0x31)            /* check sum control register */
#define TCSCR_UDPCS_ENABLE (1 << 2)
#define TCSCR_TCPCS_ENABLE (1 << 1)
#define TCSCR_IPCS_ENABLE  (1 << 0)

/* 0x32 */
// #define _DM9051_RCSSR        (0x32)            /* receive check sum status register */
#define RCSSR_UDPS  (1 << 7)
#define RCSSR_TCPS  (1 << 6)
#define RCSSR_IPS   (1 << 5)
#define RCSSR_UDPP  (1 << 4)
#define RCSSR_TCPP  (1 << 3)
#define RCSSR_IPP   (1 << 2)
#define RCSSR_RCSEN (1 << 1) // Receive Checksum Checking Enable
#define RCSSR_DCSE  (1 << 0) // Discard Checksum Error Packet

/* rxb */
#define RXB_ERR (1 << 1)
#define RXB_RDY (1 << 0)

/* 0x3D */
/* Pause Packet Control Register - default = 1 */
#define PPCR_PAUSE_COUNT 0xF // 0x08

/* 0x54 */
#define IPCOCR_CLKOUT   (1 << 7)
#define IPCOCR_DUTY_LEN 1

/* 0x57 */
/* LEDMode Control Register - LEDMode1 */
/* Value 0x81 : bit[7] = 1, bit[2] = 0, bit[1:0] = 01b */
#define LMCR_NEWMOD (1 << 7)
#define LMCR_TYPED1 (1 << 1)
#define LMCR_TYPED0 (1 << 0)
#define LMCR_MODE1  (LMCR_NEWMOD | LMCR_TYPED0)

/* 0x5E */
#define MBNDRY_WORD 0
#define MBNDRY_BYTE (1 << 7)
#define MBNDRY_DEFAULT      MBNDRY_WORD

/* 0x7E */
#define ISR_PR (1 << 0)

/* 0x7F */
#define IMR_PAR         (1 << 7)
#define IMR_PRM         (1 << 0)
#define IMR_INT_DEFAULT (IMR_PAR | IMR_PRM)
#define IMR_POL_DEFAULT (IMR_PAR)

/* Pack */
#define RSR_ERR_BITS  (RSR_RF | RSR_LCS | RSR_RWTO | RSR_AE | RSR_CE | RSR_FOE) /* | RSR_PLE */
#define BPTR_DEFAULT  (0x3f)
#define FCTR_DEAFULT  (0x38)
#define FCR_DEFAULT   (FCR_TXPEN | FCR_BKPA | FCR_BKPM | FCR_FLCE)
// #define FCR_DEFAULT1         (0x39)
// #define FCR_DEFAULT_CONF     FCR_DEFAULT
#define SMCR_DEFAULT  (0x0)
#define PBCR_MAXDRIVE (0x44)

#define PHY_STATUS_REG (0x01) /*!< basic mode status register */
#define DM9051_MRCMD   (0x72)
#define DM9051_MWCMD   (0x78)
#define OPC_REG_W      0x80 // Register Write
#define OPC_REG_R      0x00 // Register Read

#define INPUT_MODE_POLL             0
#define INPUT_MODE_INTERRUPT        1
#define INPUT_MODE_INTERRUPT_CLKOUT 2

#endif /*_DM9051_*/
