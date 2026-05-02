#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_DIR}"

source ./zephyr-env.sh

if [ ! -f build/zephyr/zephyr.bin ]; then
	echo "No build/zephyr/zephyr.bin found. Run ./scripts/build.sh first." >&2
	exit 1
fi

if pgrep -x openocd >/dev/null 2>&1; then
	echo "OpenOCD is still running and may be holding ST-LINK." >&2
	echo "Close the old flashing terminal or run: pkill -9 openocd" >&2
	exit 1
fi

OPENOCD_ADAPTER_KHZ="${OPENOCD_ADAPTER_KHZ:-50}"
OPENOCD_VERIFY="${OPENOCD_VERIFY:-1}"

echo "Slow OpenOCD flashing mode."
echo "SWD speed: ${OPENOCD_ADAPTER_KHZ} kHz"
echo "This can take 5-15 minutes after 'falling back to single memory accesses'."
echo "Do not press Ctrl-C unless you want to abort flashing."

OPENOCD="${ZEPHYR_SDK_INSTALL_DIR}/sysroots/x86_64-pokysdk-linux/usr/bin/openocd"
OPENOCD_SCRIPTS="${ZEPHYR_SDK_INSTALL_DIR}/sysroots/x86_64-pokysdk-linux/usr/share/openocd/scripts"

if [ ! -x "${OPENOCD}" ]; then
	OPENOCD="openocd"
	OPENOCD_SCRIPTS=""
fi

OPENOCD_ARGS=(
	-f boards/arm/dji_f407igh6_c/support/openocd_no_workarea.cfg
	-c "adapter speed ${OPENOCD_ADAPTER_KHZ}"
	-c "init"
	-c "targets"
	-c "halt"
	-c "flash probe 0"
	-c "stm32f2x options_read 0"
	-c "flash write_image erase unlock build/zephyr/zephyr.bin 0x08000000 bin"
	-c "shutdown"
)

if [ "${OPENOCD_VERIFY}" != "0" ]; then
	OPENOCD_ARGS=(
		"${OPENOCD_ARGS[@]:0:${#OPENOCD_ARGS[@]}-1}"
		-c "verify_image build/zephyr/zephyr.bin 0x08000000 bin"
		-c "shutdown"
	)
fi

if [ -n "${OPENOCD_SCRIPTS}" ]; then
	"${OPENOCD}" -s boards/arm/dji_f407igh6_c/support -s "${OPENOCD_SCRIPTS}" "${OPENOCD_ARGS[@]}"
else
	"${OPENOCD}" "${OPENOCD_ARGS[@]}"
fi
