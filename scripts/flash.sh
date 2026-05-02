#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_DIR}"

source ./zephyr-env.sh

if [ ! -f build/zephyr/zephyr.elf ]; then
	echo "No build/zephyr/zephyr.elf found. Run ./scripts/build.sh first." >&2
	exit 1
fi

if command -v lsusb >/dev/null 2>&1; then
	if ! lsusb | grep -Eiq '0483:374[48bf]|0483:375[23]|STMicroelectronics.*ST-LINK'; then
		echo "No ST-LINK was found by lsusb." >&2
		echo "Check ST-LINK USB connection, C board power, and USB pass-through if using a VM/WSL." >&2
		echo "Run ./scripts/check_devices.sh for a quick device report." >&2
		exit 1
	fi
fi

if ! west flash --runner openocd -- --cmd-reset-halt halt --verify; then
	echo "" >&2
	echo "Flash failed. Common causes:" >&2
	echo "1. ST-LINK is not plugged in or not passed through to Linux." >&2
	echo "2. C board has no power, or SWDIO/SWCLK/GND are wired incorrectly." >&2
	echo "3. Linux permission problem. Try: sudo ./scripts/openocd_flash.sh" >&2
	echo "4. If OpenOCD says 'timed out while waiting for target halted', connect NRST or use ./scripts/openocd_flash.sh." >&2
	exit 1
fi
