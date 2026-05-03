#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_DIR}"

usage() {
	cat <<'USAGE'
Usage:
  ./scripts/openocd_flash_dap_wl_esp.sh

This script is for the "装甲版高速无线 DAPLINK" style debugger described
in docs/装甲版无线仿真器说明书.pdf. It is not a Wi-Fi/IP debugger:
plug the Host side into this computer by USB, pair it with the Slave side,
then OpenOCD sees it as a local CMSIS-DAP adapter.

Environment:
  CMSIS_DAP_BACKEND=<auto|hid|usb_bulk>  CMSIS-DAP USB backend. Default: auto.
  CMSIS_DAP_SERIAL=<serial>              Optional adapter serial filter.
  OPENOCD_ADAPTER_KHZ=<khz>              SWD speed. Default: 100.
  OPENOCD_WORKAREASIZE=<bytes>           STM32 flash work area. Default: 0x1000.
  OPENOCD_RESET_CONFIG=<config>          Optional reset wiring config override.
  OPENOCD_PRE_RESET_HALT=0               Skip reset halt before flashing. Default: 1.
  OPENOCD_VERIFY=1                       Verify flash after write. Default: 0.
  OPENOCD_RESET_RUN=0                    Skip reset run after flashing. Default: 1.
  OPENOCD=/path/to/openocd               OpenOCD binary. Default: system openocd.
  OPENOCD_SCRIPTS=/path/scripts          Optional OpenOCD scripts directory.

Examples:
  ./scripts/openocd_flash_dap_wl_esp.sh
  CMSIS_DAP_BACKEND=hid OPENOCD_ADAPTER_KHZ=50 ./scripts/openocd_flash_dap_wl_esp.sh
  OPENOCD_VERIFY=1 ./scripts/openocd_flash_dap_wl_esp.sh
USAGE
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
	usage
	exit 0
fi

source ./zephyr-env.sh

CMSIS_DAP_BACKEND="${CMSIS_DAP_BACKEND:-auto}"
CMSIS_DAP_SERIAL="${CMSIS_DAP_SERIAL:-}"
OPENOCD_ADAPTER_KHZ="${OPENOCD_ADAPTER_KHZ:-100}"
OPENOCD_WORKAREASIZE="${OPENOCD_WORKAREASIZE:-0x1000}"
OPENOCD_RESET_CONFIG="${OPENOCD_RESET_CONFIG:-}"
OPENOCD_PRE_RESET_HALT="${OPENOCD_PRE_RESET_HALT:-1}"
OPENOCD_VERIFY="${OPENOCD_VERIFY:-0}"
OPENOCD_RESET_RUN="${OPENOCD_RESET_RUN:-1}"

if [ ! -f build/zephyr/zephyr.bin ]; then
	echo "No build/zephyr/zephyr.bin found. Run ./scripts/build.sh first." >&2
	exit 1
fi

if pgrep -x openocd >/dev/null 2>&1; then
	echo "OpenOCD is already running and may hold GDB/Telnet ports." >&2
	echo "Close it or run: pkill -9 openocd" >&2
	exit 1
fi

if [ -n "${OPENOCD:-}" ]; then
	OPENOCD_BIN="${OPENOCD}"
elif command -v openocd >/dev/null 2>&1; then
	OPENOCD_BIN="$(command -v openocd)"
else
	OPENOCD_BIN="${ZEPHYR_SDK_INSTALL_DIR}/sysroots/x86_64-pokysdk-linux/usr/bin/openocd"
fi

if [ ! -x "${OPENOCD_BIN}" ]; then
	echo "OpenOCD not found or not executable: ${OPENOCD_BIN}" >&2
	exit 1
fi

OPENOCD_SCRIPT_ARGS=()
if [ -n "${OPENOCD_SCRIPTS:-}" ]; then
	OPENOCD_SCRIPT_ARGS=(-s "${OPENOCD_SCRIPTS}")
elif [ "${OPENOCD_BIN}" = "${ZEPHYR_SDK_INSTALL_DIR}/sysroots/x86_64-pokysdk-linux/usr/bin/openocd" ]; then
	OPENOCD_SCRIPT_ARGS=(-s "${ZEPHYR_SDK_INSTALL_DIR}/sysroots/x86_64-pokysdk-linux/usr/share/openocd/scripts")
fi

OPENOCD_ARGS=(
	-f interface/cmsis-dap.cfg
)

if [ "${CMSIS_DAP_BACKEND}" != "auto" ]; then
	OPENOCD_ARGS+=(-c "cmsis_dap_backend ${CMSIS_DAP_BACKEND}")
fi

if [ -n "${CMSIS_DAP_SERIAL}" ]; then
	OPENOCD_ARGS+=(-c "adapter serial ${CMSIS_DAP_SERIAL}")
fi

OPENOCD_ARGS+=(
	-c "transport select swd"
	-c "set WORKAREASIZE ${OPENOCD_WORKAREASIZE}"
	-f target/stm32f4x.cfg
	-c "adapter speed ${OPENOCD_ADAPTER_KHZ}"
	-c "init"
	-c "targets"
)

if [ -n "${OPENOCD_RESET_CONFIG}" ]; then
	OPENOCD_ARGS+=(-c "reset_config ${OPENOCD_RESET_CONFIG}")
fi

if [ "${OPENOCD_PRE_RESET_HALT}" != "0" ]; then
	OPENOCD_ARGS+=(-c "reset halt")
fi

OPENOCD_ARGS+=(
	-c "halt"
	-c "flash probe 0"
	-c "stm32f2x options_read 0"
	-c "flash write_image erase unlock build/zephyr/zephyr.bin 0x08000000 bin"
)

if [ "${OPENOCD_VERIFY}" != "0" ]; then
	OPENOCD_ARGS+=(-c "verify_image build/zephyr/zephyr.bin 0x08000000 bin")
fi

if [ "${OPENOCD_RESET_RUN}" != "0" ]; then
	OPENOCD_ARGS+=(-c "reset run")
fi

OPENOCD_ARGS+=(-c "shutdown")

echo "Wireless DAPLink CMSIS-DAP OpenOCD flashing mode."
echo "Backend: ${CMSIS_DAP_BACKEND}"
echo "SWD speed: ${OPENOCD_ADAPTER_KHZ} kHz"
echo "Work area: ${OPENOCD_WORKAREASIZE}"
echo "Reset config: ${OPENOCD_RESET_CONFIG:-target default}"
echo "Pre reset halt: ${OPENOCD_PRE_RESET_HALT}"
echo "Verify: ${OPENOCD_VERIFY}"
echo "Reset run: ${OPENOCD_RESET_RUN}"

"${OPENOCD_BIN}" "${OPENOCD_SCRIPT_ARGS[@]}" "${OPENOCD_ARGS[@]}"
