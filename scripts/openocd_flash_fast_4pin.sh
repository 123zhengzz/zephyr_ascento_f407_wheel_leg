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

OPENOCD_ADAPTER_KHZ="${OPENOCD_ADAPTER_KHZ:-100}"
OPENOCD_WORKAREASIZE="${OPENOCD_WORKAREASIZE:-0x1000}"
OPENOCD_VERIFY="${OPENOCD_VERIFY:-0}"

echo "Fast 4-pin SWD OpenOCD flashing mode."
echo "SWD speed: ${OPENOCD_ADAPTER_KHZ} kHz"
echo "Work area: ${OPENOCD_WORKAREASIZE}"
echo "Verify: ${OPENOCD_VERIFY}"
echo "No NRST is used; power-cycle the board manually after flashing."

OPENOCD="${ZEPHYR_SDK_INSTALL_DIR}/sysroots/x86_64-pokysdk-linux/usr/bin/openocd"
OPENOCD_SCRIPTS="${ZEPHYR_SDK_INSTALL_DIR}/sysroots/x86_64-pokysdk-linux/usr/share/openocd/scripts"

if [ ! -x "${OPENOCD}" ]; then
	OPENOCD="openocd"
	OPENOCD_SCRIPTS=""
fi

OPENOCD_ARGS=(
	-f interface/stlink.cfg
	-c "transport select hla_swd"
	-c "set WORKAREASIZE ${OPENOCD_WORKAREASIZE}"
	-f target/stm32f4x.cfg
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
	"${OPENOCD}" -s "${OPENOCD_SCRIPTS}" "${OPENOCD_ARGS[@]}"
else
	"${OPENOCD}" "${OPENOCD_ARGS[@]}"
fi
