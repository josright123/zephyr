# pico-dhcp-run Persistent Profile Setup (GPT-5.3)

## Goal

Make pico-dhcp-run available in every new PowerShell terminal.

## Step 1: Add function to PowerShell profile (one line)

```powershell
if (!(Test-Path $PROFILE)) { New-Item -Type File -Path $PROFILE -Force | Out-Null }; Add-Content $PROFILE "`nfunction pico-dhcp-run { Set-Location 'C:/Users/joseph/.pico-sdk/zephyr_workspace/zephyr-main/samples/net/dhcpv4_client'; & 'C:/Users/joseph/scoop/apps/python313/current/python.exe' -m pip install --upgrade jsonschema pyelftools; & west build -p always -b rpi_pico . -- '-DDTC_OVERLAY_FILE=boards/rpi_pico.overlay' '-DEXTRA_CONF_FILE=overlay-dm9051.conf'; & 'C:/Users/joseph/.pico-sdk/picotool/2.2.0-a4/picotool/picotool.exe' info *> `$null; if (`$LASTEXITCODE -ne 0) { Write-Host 'Pico not found in BOOTSEL mode. Hold BOOTSEL and reconnect USB, then run pico-dhcp-run again.' -ForegroundColor Yellow; return }; & 'C:/Users/joseph/.pico-sdk/picotool/2.2.0-a4/picotool/picotool.exe' load build/zephyr/zephyr.elf -fx }"
```

## Step 2: Load profile now (no new terminal needed)

```powershell
. $PROFILE
```

## Step 3: Run

```powershell
pico-dhcp-run
```

## Notes

- This appends the function to your profile and keeps existing profile content.
- The function checks BOOTSEL visibility before flashing and prints guidance if needed.
- This avoids OpenOCD CMSIS-DAP probe requirements.
