# Zephyr on Raspberry Pi Pico with ETH_DM9051 (SPI)

This guide explains how to build, flash, run, and verify:

- Zephyr sample: samples/net/dhcpv4_client
- Board: rpi_pico
- Ethernet controller: ETH_DM9051 (SPI interface)

## 1) Prerequisites

1. Zephyr workspace is already set up under:
   - C:/Users/joseph/.pico-sdk/zephyr_workspace/zephyr-main
2. A DM9051 SPI Ethernet module is wired to Raspberry Pi Pico.
3. The network side has a DHCP server (router or test DHCP server).

## 2) Devicetree Overlay

Create file:

- samples/net/dhcpv4_client/boards/rpi_pico.overlay

Use this content:

```dts
#include <zephyr/dt-bindings/gpio/gpio.h>
#include <zephyr/dt-bindings/pinctrl/rpi-pico-rp2040-pinctrl.h>

/* SPI0: MISO=16, CS=17, SCK=18, MOSI=19
 * DM9051 INT=20 (active low), RESET=21 (active low)
 */

&pinctrl {
	spi0_dm9051_default: spi0_dm9051_default {
		group1 {
			pinmux = <SPI0_RX_P16>, <SPI0_SCK_P18>, <SPI0_TX_P19>;
		};
	};
};

&spi0 {
	pinctrl-0 = <&spi0_dm9051_default>;
	pinctrl-names = "default";
	cs-gpios = <&gpio0 17 GPIO_ACTIVE_LOW>;
	status = "okay";

	dm9051: ethernet@0 {
		compatible = "davicom,dm9051";
		reg = <0>;
		spi-max-frequency = <12000000>;
		int-gpios = <&gpio0 20 GPIO_ACTIVE_LOW>;
		reset-gpios = <&gpio0 21 GPIO_ACTIVE_LOW>;
		status = "okay";
	};
};
```

## 3) Kconfig Overlay

Create file:

- samples/net/dhcpv4_client/overlay-dm9051.conf

Use this content:

```conf
CONFIG_NET_L2_ETHERNET=y
CONFIG_ETH_DRIVER=y
CONFIG_ETH_DM9051=y
CONFIG_LOG=y
CONFIG_ETHERNET_LOG_LEVEL_DBG=y
```

## 4) Build

Open terminal and switch to Zephyr workspace root first:

```powershell
cd C:/Users/joseph/.pico-sdk/zephyr_workspace/zephyr-main
```

Build command:

```powershell
west build -p always -b rpi_pico samples/net/dhcpv4_client -- -DDTC_OVERLAY_FILE=samples/net/dhcpv4_client/boards/rpi_pico.overlay -DEXTRA_CONF_FILE=samples/net/dhcpv4_client/overlay-dm9051.conf
```

If SPI is unstable in your setup, lower speed by editing overlay:

- spi-max-frequency = <8000000>;

## 5) Flash

### Option A: west flash

```powershell
west flash
```

### Option B: picotool (recommended if west flash fails)

```powershell
C:/Users/joseph/.pico-sdk/picotool/2.2.0-a4/picotool/picotool.exe load C:/Users/joseph/.pico-sdk/zephyr_workspace/zephyr-main/build/zephyr/zephyr.elf -fx
```

### Option C: UF2 drag and drop

Use file:

- build/zephyr/zephyr.uf2

## 6) Run and Verify DHCP

1. Connect DM9051 Ethernet to a DHCP-enabled network.
2. Open serial terminal at 115200.
3. Expected logs include messages similar to:
   - Run dhcpv4 client
   - Start on ethernet: index=1
   - Received: x.x.x.x
   - Address/Subnet/Router/Lease time

## 7) Typical Troubleshooting

1. No IP obtained:
   - Check DHCP server availability and cable/link LEDs.
2. DM9051 not responding:
   - Verify SPI wires (MISO/MOSI/SCK/CS) and common GND.
3. Interrupt issue:
   - int-gpios must be active low.
4. Frequent packet errors:
   - Reduce spi-max-frequency to 8 MHz.
5. Command west topdir fails:
   - Ensure terminal current directory is inside Zephyr west workspace,
     such as C:/Users/joseph/.pico-sdk/zephyr_workspace/zephyr-main.

## 8) Quick Wiring Reference

- DM9051 SPI MISO -> Pico GPIO16
- DM9051 SPI CS   -> Pico GPIO17
- DM9051 SPI SCK  -> Pico GPIO18
- DM9051 SPI MOSI -> Pico GPIO19
- DM9051 INT      -> Pico GPIO20 (active low)
- DM9051 RESET    -> Pico GPIO21 (active low)

