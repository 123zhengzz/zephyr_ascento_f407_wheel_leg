#!/usr/bin/env bash
# Source this file from the project root before using west manually:
#   source ./zephyr-env.sh

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ZEPHYR_WORKSPACE="/home/h/code/zephyr_robot"
ZEPHYR_VENV="${ZEPHYR_WORKSPACE}/.venv"
ZEPHYR_BASE="${ZEPHYR_WORKSPACE}/zephyr"
ZEPHYR_SDK_INSTALL_DIR="/home/h/zephyr-sdk-0.17.0"

if [ ! -f "${ZEPHYR_VENV}/bin/activate" ]; then
	echo "Missing Zephyr Python venv: ${ZEPHYR_VENV}" >&2
	return 1 2>/dev/null || exit 1
fi

if [ ! -f "${ZEPHYR_BASE}/zephyr-env.sh" ]; then
	echo "Missing Zephyr tree: ${ZEPHYR_BASE}" >&2
	return 1 2>/dev/null || exit 1
fi

if [ ! -d "${ZEPHYR_SDK_INSTALL_DIR}" ]; then
	echo "Missing Zephyr SDK: ${ZEPHYR_SDK_INSTALL_DIR}" >&2
	return 1 2>/dev/null || exit 1
fi

source "${ZEPHYR_VENV}/bin/activate"
source "${ZEPHYR_BASE}/zephyr-env.sh"

export ZEPHYR_BASE
export ZEPHYR_SDK_INSTALL_DIR
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr

cd "${PROJECT_DIR}" || return 1 2>/dev/null || exit 1

echo "Zephyr environment ready."
echo "Project: ${PROJECT_DIR}"
echo "Zephyr:  ${ZEPHYR_BASE}"
echo "SDK:     ${ZEPHYR_SDK_INSTALL_DIR}"
