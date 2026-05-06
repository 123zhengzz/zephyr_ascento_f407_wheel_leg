#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_DIR}"

source ./zephyr-env.sh

if [ ! -f build/zephyr/zephyr.elf ]; then
	echo "No build/zephyr/zephyr.elf found. Run ./scripts/build.sh first." >&2
	exit 1
fi

# 查找 OpenOCD
if command -v openocd >/dev/null 2>&1; then
    OPENOCD="$(command -v openocd)"
elif [ -n "${ZEPHYR_SDK_INSTALL_DIR:-}" ]; then
    OPENOCD="${ZEPHYR_SDK_INSTALL_DIR}/sysroots/x86_64-pokysdk-linux/usr/bin/openocd"
else
    OPENOCD="openocd"
fi

if [ ! -x "${OPENOCD}" ]; then
    echo "[错误] 找不到 openocd" >&2
    exit 1
fi

# 查找 OpenOCD 脚本目录
ZEPHYR_BASE="$(west topdir 2>/dev/null || echo "")"
if [ -n "${ZEPHYR_BASE}" ]; then
    OPENOCD_SCRIPTS="${ZEPHYR_BASE}/../modules/hal/openocd/scripts"
    if [ ! -d "${OPENOCD_SCRIPTS}" ]; then
        OPENOCD_SCRIPTS=""
    fi
fi

OPENOCD_ARGS=(
    -f boards/arm/dji_f407igh6_c/support/openocd_cmsis_dap.cfg
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
