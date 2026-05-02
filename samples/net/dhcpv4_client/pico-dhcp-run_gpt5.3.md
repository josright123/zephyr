# pico-dhcp-run Quick Use (GPT-5.3)

## Goal

Copy the Current terminal only one-liner from the guide, paste it into PowerShell, then run `pico-dhcp-run`.

## Steps

1. Open PowerShell in:
   C:/Users/joseph/.pico-sdk/zephyr_workspace/zephyr-main/samples/net/dhcpv4_client

2. Paste this one-liner (Current terminal only):

```powershell
Set-Item -Path Function:pico-cdc-port -Value { $ports = Get-PnpDevice -Class Ports -ErrorAction SilentlyContinue | Where-Object { $_.FriendlyName -match 'USB Serial Device|CDC|COM' }; if ($ports) { $ports | Select-Object FriendlyName, InstanceId } else { Write-Host 'No active COM port detected yet.' -ForegroundColor Yellow } }
Set-Item -Path Function:pico-dhcp-run -Value { Set-Location 'C:/Users/joseph/.pico-sdk/zephyr_workspace/zephyr-main/samples/net/dhcpv4_client'; & 'C:/Users/joseph/scoop/apps/python313/current/python.exe' -m pip install --upgrade jsonschema pyelftools; & west build -p always -b rpi_pico . -- '-DDTC_OVERLAY_FILE=boards/rpi_pico.overlay' '-DEXTRA_CONF_FILE=overlay-dm9051.conf'; & 'C:/Users/joseph/.pico-sdk/picotool/2.2.0-a4/picotool/picotool.exe' info *> $null; if ($LASTEXITCODE -ne 0) { Write-Host 'Pico not found in BOOTSEL mode. Hold BOOTSEL and reconnect USB, then run pico-dhcp-run again.' -ForegroundColor Yellow; return }; & 'C:/Users/joseph/.pico-sdk/picotool/2.2.0-a4/picotool/picotool.exe' load build/zephyr/zephyr.elf -fx; Write-Host 'Available serial ports:' -ForegroundColor Cyan; pico-cdc-port }
```

3. Run:

```powershell
pico-dhcp-run
```

## Notes

- This function is only available in the current terminal session.
- The function checks BOOTSEL visibility before flashing and prints guidance if needed.
- It prints a likely COM port list after flashing.
- This avoids OpenOCD CMSIS-DAP probe requirements.
