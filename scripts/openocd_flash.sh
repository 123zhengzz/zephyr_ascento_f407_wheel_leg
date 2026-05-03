#!/usr/bin/env bash
# DISABLED: openocd.cfg 无 workarea 配置导致 flash write 算法超时。
# 请使用 ./scripts/flash.sh 或 ./scripts/openocd_mass_erase_flash.sh
echo "openocd_flash.sh has been disabled (flash write timeout). Use ./scripts/flash.sh instead." >&2
exit 1

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_DIR}"

source ./zephyr-env.sh

if [ ! -f build/zephyr/zephyr.elf ]; then
	echo "No build/zephyr/zephyr.elf found. Run ./scripts/build.sh first." >&2
	exit 1
fi

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
	-c "flash write_image erase unlock build/zephyr/zephyr.hex"
	-c "verify_image build/zephyr/zephyr.hex"
	-c "reset run"
	-c "shutdown"
)

if [ -n "${OPENOCD_SCRIPTS}" ]; then
	"${OPENOCD}" -s boards/arm/dji_f407igh6_c/support -s "${OPENOCD_SCRIPTS}" "${OPENOCD_ARGS[@]}"
else
	"${OPENOCD}" "${OPENOCD_ARGS[@]}"
fi
