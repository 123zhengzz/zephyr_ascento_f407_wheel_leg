#!/usr/bin/env bash
set -euo pipefail

PORT="${1:-auto}"
COUNT="${2:-20}"
BAUD="${3:-115200}"

if [ "${PORT}" = "auto" ]; then
	PORT="$(ls /dev/serial/by-id/* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n 1 || true)"
fi

if [ -z "${PORT}" ] || [ ! -e "${PORT}" ]; then
	echo "No USB serial port found." >&2
	exit 1
fi

if [ ! -r "${PORT}" ] || [ ! -w "${PORT}" ]; then
	echo "No permission to access ${PORT}." >&2
	echo "Temporary fix:" >&2
	echo "  sudo chmod a+rw ${PORT}" >&2
	echo "Persistent fix, then log out/in:" >&2
	echo "  sudo usermod -aG dialout ${USER}" >&2
	exit 1
fi

if ! [[ "${COUNT}" =~ ^[0-9]+$ ]] || [ "${COUNT}" -lt 3 ]; then
	echo "COUNT must be an integer >= 3." >&2
	exit 1
fi

TMP="$(mktemp)"
cleanup() {
	rm -f "${TMP}"
}
trap cleanup EXIT

stty -F "${PORT}" "${BAUD}" cs8 -cstopb -parenb -ixon -ixoff -echo raw

timeout 0.2 cat "${PORT}" >/dev/null 2>/dev/null || true

timeout 8 cat "${PORT}" >"${TMP}" &
READER_PID=$!

printf '\r\nrobot enable 0\r\nrobot stop\r\nmotor debug stop\r\n' >"${PORT}"
sleep 0.2

for _ in $(seq 1 "${COUNT}"); do
	printf 'robot status\r\n' >"${PORT}"
	sleep 0.12
done

sleep 0.5
kill "${READER_PID}" 2>/dev/null || true
wait "${READER_PID}" 2>/dev/null || true

PITCHES="$(sed -n 's/.*pitch=\([-+0-9.]*\).*/\1/p' "${TMP}")"

if [ -z "${PITCHES}" ]; then
	echo "No pitch readings found. Raw serial output:" >&2
	sed -n '1,80p' "${TMP}" >&2
	exit 1
fi

printf '%s\n' "${PITCHES}" | awk '
	BEGIN { n = 0; }
	{
		v = $1 + 0.0;
		if (n == 0 || v < min) min = v;
		if (n == 0 || v > max) max = v;
		sum += v;
		n++;
	}
	END {
		avg = sum / n;
		printf("IMU pitch zero samples: %d\n", n);
		printf("pitch avg: %.3f deg\n", avg);
		printf("pitch min/max: %.3f / %.3f deg\n", min, max);
		printf("Suggested app_config.h value:\n");
		printf("#define APP_ANGLE_ZERO_DEG %.3ff\n", avg);
	}
'
