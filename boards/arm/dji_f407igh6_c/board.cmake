board_runner_args(openocd "--config=${BOARD_DIR}/support/openocd.cfg")
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
