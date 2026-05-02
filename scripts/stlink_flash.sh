#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_DIR}"

if [ ! -f build/zephyr/zephyr.bin ]; then
	echo "No build/zephyr/zephyr.bin found. Run ./scripts/build.sh first." >&2
	exit 1
fi

if pgrep -x openocd >/dev/null 2>&1; then
	echo "OpenOCD is still running and may be holding ST-LINK." >&2
	echo "Close the old flashing terminal or run: pkill -9 openocd" >&2
	exit 1
fi

if ! command -v st-flash >/dev/null 2>&1; then
	echo "st-flash was not found." >&2
	echo "Install it with: sudo apt install stlink-tools" >&2
	exit 1
fi

if command -v st-info >/dev/null 2>&1; then
	st-info --probe
fi

MODE="${STLINK_MODE:-normal}"
FREQ_KHZ="${STLINK_FREQ_KHZ:-100}"

ST_FLASH_ARGS=(--reset "--freq=${FREQ_KHZ}")

case "${MODE}" in
normal)
	;;
hotplug)
	ST_FLASH_ARGS=(--hot-plug "${ST_FLASH_ARGS[@]}")
	;;
under-reset)
	ST_FLASH_ARGS=(--connect-under-reset "${ST_FLASH_ARGS[@]}")
	;;
*)
	echo "Unsupported STLINK_MODE='${MODE}'." >&2
	echo "Use one of: normal, hotplug, under-reset" >&2
	exit 1
	;;
esac

echo "Flashing with st-flash mode=${MODE}, freq=${FREQ_KHZ} kHz"
if ! st-flash "${ST_FLASH_ARGS[@]}" write build/zephyr/zephyr.bin 0x08000000; then
	echo "" >&2
	echo "st-flash failed. If the log says 'Flash loader write error', try:" >&2
	echo "  STLINK_MODE=hotplug ./scripts/stlink_flash.sh" >&2
	echo "  STLINK_MODE=hotplug STLINK_FREQ_KHZ=50 ./scripts/stlink_flash.sh" >&2
	echo "  STLINK_MODE=under-reset ./scripts/stlink_flash.sh   # only if board NRST is wired separately" >&2
	echo "  ./scripts/openocd_flash_bin_slow.sh" >&2
	exit 1
fi
