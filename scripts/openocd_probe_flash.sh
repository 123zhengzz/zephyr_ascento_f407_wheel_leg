#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_DIR}"

source ./zephyr-env.sh

OPENOCD="${ZEPHYR_SDK_INSTALL_DIR}/sysroots/x86_64-pokysdk-linux/usr/bin/openocd"
OPENOCD_SCRIPTS="${ZEPHYR_SDK_INSTALL_DIR}/sysroots/x86_64-pokysdk-linux/usr/share/openocd/scripts"

if [ ! -x "${OPENOCD}" ]; then
	OPENOCD="openocd"
	OPENOCD_SCRIPTS=""
fi

OPENOCD_ARGS=(
	-f boards/arm/dji_f407igh6_c/support/openocd.cfg
	-c "init"
	-c "targets"
	-c "halt"
	-c "flash probe 0"
	-c "stm32f2x options_read 0"
	-c "flash info 0"
	-c "flash banks"
	-c "mdw 0x40023c10 4"
	-c "mdw 0x40023c14 4"
	-c "mdw 0x08000000 8"
	-c "shutdown"
)

if [ -n "${OPENOCD_SCRIPTS}" ]; then
	"${OPENOCD}" -s boards/arm/dji_f407igh6_c/support -s "${OPENOCD_SCRIPTS}" "${OPENOCD_ARGS[@]}"
else
	"${OPENOCD}" "${OPENOCD_ARGS[@]}"
fi
