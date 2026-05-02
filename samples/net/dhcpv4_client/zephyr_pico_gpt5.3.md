# Zephyr on Raspberry Pi Pico with ETH_DM9051 (SPI)

This guide explains how to build, flash, run, and verify:

- Zephyr sample: samples/net/dhcpv4_client
- Board: rpi_pico
- Ethernet controller: ETH_DM9051 (SPI interface)

## Quickest 3-Command Flow (Windows PowerShell)

```powershell
cd C:/Users/joseph/.pico-sdk/zephyr_workspace/zephyr-main/samples/net/dhcpv4_client ; C:/Users/joseph/scoop/apps/python313/current/python.exe -m pip install --upgrade jsonschema pyelftools ; west build -p always -b rpi_pico . --% -- -DDTC_OVERLAY_FILE=boards/rpi_pico.overlay -DEXTRA_CONF_FILE=overlay-dm9051.conf ; C:/Users/joseph/.pico-sdk/picotool/2.2.0-a4/picotool/picotool.exe load build/zephyr/zephyr.elf -fx
```

## One-Line Alias (Paste Once, Then Run Fast)

Current terminal only (paste once, then use `pico-dhcp-run` anytime in this session):

```powershell
Set-Item -Path Function:pico-dhcp-run -Value { Set-Location 'C:/Users/joseph/.pico-sdk/zephyr_workspace/zephyr-main/samples/net/dhcpv4_client'; & 'C:/Users/joseph/scoop/apps/python313/current/python.exe' -m pip install --upgrade jsonschema pyelftools; & west build -p always -b rpi_pico . -- '-DDTC_OVERLAY_FILE=boards/rpi_pico.overlay' '-DEXTRA_CONF_FILE=overlay-dm9051.conf'; & 'C:/Users/joseph/.pico-sdk/picotool/2.2.0-a4/picotool/picotool.exe' info *> $null; if ($LASTEXITCODE -ne 0) { Write-Host 'Pico not found in BOOTSEL mode. Hold BOOTSEL and reconnect USB, then run pico-dhcp-run again.' -ForegroundColor Yellow; return }; & 'C:/Users/joseph/.pico-sdk/picotool/2.2.0-a4/picotool/picotool.exe' load build/zephyr/zephyr.elf -fx }
```

Run it:

```powershell
pico-dhcp-run
```

Persistent alias (survives new terminal windows):

```powershell
if (!(Test-Path $PROFILE)) { New-Item -Type File -Path $PROFILE -Force | Out-Null }; Add-Content $PROFILE "`nfunction pico-dhcp-run { Set-Location 'C:/Users/joseph/.pico-sdk/zephyr_workspace/zephyr-main/samples/net/dhcpv4_client'; & 'C:/Users/joseph/scoop/apps/python313/current/python.exe' -m pip install --upgrade jsonschema pyelftools; & west build -p always -b rpi_pico . -- '-DDTC_OVERLAY_FILE=boards/rpi_pico.overlay' '-DEXTRA_CONF_FILE=overlay-dm9051.conf'; & 'C:/Users/joseph/.pico-sdk/picotool/2.2.0-a4/picotool/picotool.exe' info *> `$null; if (`$LASTEXITCODE -ne 0) { Write-Host 'Pico not found in BOOTSEL mode. Hold BOOTSEL and reconnect USB, then run pico-dhcp-run again.' -ForegroundColor Yellow; return }; & 'C:/Users/joseph/.pico-sdk/picotool/2.2.0-a4/picotool/picotool.exe' load build/zephyr/zephyr.elf -fx }"
```

The function now checks BOOTSEL visibility before flashing and prints guidance if
the board is not detectable.

This avoids the CMSIS-DAP requirement of `west flash` in probe-less setups.

## 1) Prerequisites

1. Zephyr workspace is already set up under:
   - C:/Users/joseph/.pico-sdk/zephyr_workspace/zephyr-main
2. A DM9051 SPI Ethernet module is wired to Raspberry Pi Pico.
3. The network side has a DHCP server (router or test DHCP server).
4. Python dependencies required by this Zephyr workspace are installed in the
	same Python used by west:

```powershell
C:/Users/joseph/scoop/apps/python313/current/python.exe -m pip install --upgrade jsonschema pyelftools
```

## 2) Devicetree Overlay

Create file:

- samples/net/dhcpv4_client/boards/rpi_pico.overlay

Use this content:

```dts
#include <zephyr/dt-bindings/gpio/gpio.h>
#include <zephyr/dt-bindings/pinctrl/rpi-pico-rp2040-pinctrl.h>

/* SPI0: MISO=16, CS=17, SCK=18, MOSI=19
 * DM9051 INT=20 (active low)
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

Open terminal and switch to the sample directory first:

```powershell
cd C:/Users/joseph/.pico-sdk/zephyr_workspace/zephyr-main/samples/net/dhcpv4_client
```

Build command (PowerShell):

```powershell
west build -p always -b rpi_pico . --% -- -DDTC_OVERLAY_FILE=boards/rpi_pico.overlay -DEXTRA_CONF_FILE=overlay-dm9051.conf
```

Why `--%` on Windows PowerShell?

- It prevents PowerShell from mangling arguments that include `.overlay` and
	`.conf` values.

If SPI is unstable in your setup, lower speed by editing overlay:

- spi-max-frequency = <8000000>;

## 5) Flash

### Option A: west flash

```powershell
west flash
```

### Option B: picotool (recommended if west flash fails)

```powershell
C:/Users/joseph/.pico-sdk/picotool/2.2.0-a4/picotool/picotool.exe load C:/Users/joseph/.pico-sdk/zephyr_workspace/zephyr-main/samples/net/dhcpv4_client/build/zephyr/zephyr.elf -fx
```

### Option C: UF2 drag and drop

Use file:

- samples/net/dhcpv4_client/build/zephyr/zephyr.uf2

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
6. CMake error: Missing jsonschema dependency:
	- Install with:
	  C:/Users/joseph/scoop/apps/python313/current/python.exe -m pip install jsonschema
7. Build error: ModuleNotFoundError: No module named 'elftools':
	- Install with:
	  C:/Users/joseph/scoop/apps/python313/current/python.exe -m pip install pyelftools
8. Overlay/conf path parsed incorrectly in PowerShell (e.g. `.overlay` or
	`.conf` split from file name):
	- Use `--%` in the west build command as shown in section 4.

## 8) Quick Wiring Reference

- DM9051 SPI MISO -> Pico GPIO16
- DM9051 SPI CS   -> Pico GPIO17
- DM9051 SPI SCK  -> Pico GPIO18
- DM9051 SPI MOSI -> Pico GPIO19
- DM9051 INT      -> Pico GPIO20 (active low)
- DM9051 RESET    -> Pico GPIO21 (active low)

