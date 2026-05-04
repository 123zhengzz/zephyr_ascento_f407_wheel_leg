#!/usr/bin/env bash
#===============================================================================
# Ascento F407 CMSIS-DAP 无线烧录脚本
#
# 用法:
#   ./scripts/flash_dap.sh           # 直接烧录（不重新编译）
#   ./scripts/flash_dap.sh build     # 重新编译 + 烧录
#
# 前置条件:
#   1. Horco CMSIS-DAP 无线调试器 Host 端已插入 USB
#   2. Slave 端已连接机器人 SWD 口
#   3. 机器人已上电
#===============================================================================
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_DIR}"

# ------------------------------------------------------------------
# 可选：先编译
# ------------------------------------------------------------------
if [ "${1:-}" = "build" ]; then
    echo "=== 编译中 ==="
    source ./zephyr-env.sh
    cd build && ninja
    echo "=== 编译完成 ==="
    echo ""
fi

# ------------------------------------------------------------------
# 检查固件
# ------------------------------------------------------------------
if [ ! -f build/zephyr/zephyr.bin ]; then
    echo "[错误] 找不到 build/zephyr/zephyr.bin" >&2
    echo "请先执行: ./scripts/flash_dap.sh build" >&2
    exit 1
fi

BIN_SIZE=$(stat -c%s build/zephyr/zephyr.bin)
echo "固件: build/zephyr/zephyr.bin (${BIN_SIZE} 字节)"

# ------------------------------------------------------------------
# 查找 OpenOCD
# ------------------------------------------------------------------
if command -v openocd >/dev/null 2>&1; then
    OPENOCD="$(command -v openocd)"
elif [ -n "${ZEPHYR_SDK_INSTALL_DIR:-}" ]; then
    OPENOCD="${ZEPHYR_SDK_INSTALL_DIR}/sysroots/x86_64-pokysdk-linux/usr/bin/openocd"
else
    source ./zephyr-env.sh 2>/dev/null || true
    OPENOCD="${ZEPHYR_SDK_INSTALL_DIR}/sysroots/x86_64-pokysdk-linux/usr/bin/openocd"
fi

if [ ! -x "${OPENOCD}" ]; then
    echo "[错误] 找不到 openocd" >&2
    echo "请先: source ./zephyr-env.sh" >&2
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

OPENOCD_CFG=(-f interface/cmsis-dap.cfg)
if [ -n "${OPENOCD_SCRIPTS:-}" ]; then
    OPENOCD_CFG+=(-s "${OPENOCD_SCRIPTS}")
fi

# ------------------------------------------------------------------
# 烧录
# ------------------------------------------------------------------
echo ""
echo "=== 开始烧录 ==="
echo "调试器: CMSIS-DAP (无线 DAPLink)"
echo "目标:   STM32F407IGH6 @ 0x08000000"
echo ""

"${OPENOCD}" \
    "${OPENOCD_CFG[@]}" \
    -c "transport select swd" \
    -c "adapter speed 100" \
    -f target/stm32f4x.cfg \
    -c "init" \
    -c "targets" \
    -c "reset halt" \
    -c "flash write_image erase unlock build/zephyr/zephyr.bin 0x08000000" \
    -c "reset run" \
    -c "shutdown" \
    2>&1 | grep -v "^Info : \|^Debug: "

EXIT_CODE=${PIPESTATUS[0]}

if [ ${EXIT_CODE} -eq 0 ]; then
    echo ""
    echo "=== 烧录成功 ==="
    echo ""
    echo "现在可以接串口查看日志:"
    echo "  ./scripts/serial.sh"
else
    echo ""
    echo "[错误] 烧录失败 (exit=${EXIT_CODE})" >&2
    echo ""
    echo "常见原因:" >&2
    echo "  1. 调试器 Host 端没插 USB" >&2
    echo "  2. Slave 端没连机器人 SWD" >&2
    echo "  3. 机器人没上电" >&2
    echo "  4. 另一个 openocd 进程还占着 (pkill -9 openocd)" >&2
    exit ${EXIT_CODE}
fi
