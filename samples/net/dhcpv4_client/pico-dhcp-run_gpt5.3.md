# pico-dhcp-run Quick Use (GPT-5.3)

## Goal

Copy the Current terminal only one-liner from the guide, paste it into PowerShell, then run `pico-dhcp-run`.

## Steps

1. Open PowerShell in:
   C:/Users/joseph/.pico-sdk/zephyr_workspace/zephyr-main/samples/net/dhcpv4_client

2. Paste this one-liner (Current terminal only):

```powershell
Set-Item -Path Function:pico-dhcp-run -Value { Set-Location 'C:/Users/joseph/.pico-sdk/zephyr_workspace/zephyr-main/samples/net/dhcpv4_client'; & 'C:/Users/joseph/scoop/apps/python313/current/python.exe' -m pip install --upgrade jsonschema pyelftools; & west build -p always -b rpi_pico . -- '-DDTC_OVERLAY_FILE=boards/rpi_pico.overlay' '-DEXTRA_CONF_FILE=overlay-dm9051.conf'; & 'C:/Users/joseph/.pico-sdk/picotool/2.2.0-a4/picotool/picotool.exe' load build/zephyr/zephyr.elf -fx }
```

3. Run:

```powershell
pico-dhcp-run
```

## Notes

- This function is only available in the current terminal session.
- If flashing fails, put Pico into BOOTSEL mode and run `pico-dhcp-run` again.
- This avoids OpenOCD CMSIS-DAP probe requirements.
