cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephyr_ascento_f407_wheel_leg C)

target_include_directories(app PRIVATE src)

target_sources(app PRIVATE
  src/main.c
  src/pid.c
  src/control.c
  src/ascento_balance.c
  src/dji_m3508.c
  src/dm4340.c
  src/motor_debug.c
  src/bmi088.c
  src/battery.c
  src/shell_commands.c
)

target_link_libraries(app PRIVATE m)
