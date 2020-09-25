cmake_minimum_required(VERSION 3.0)
project(
    "Self Balancing Robot"
    VERSION 0.1.0
    DESCRIPTION "A self balancing robot, controlled with an ATMega328p"
    LANGUAGES C
)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(tools/target_cmake_scripts/atmega328p.cmake)

find_library(ZARDUINO_LIB
    NAMES zarduino
    PATHS /usr/local/lib
)

add_executable(pid_oled.elf
    src/common/buffer.c
    src/common/interface_common.c
    src/common/interface_param.c
    src/common/main.c
    src/common/motors.c
    src/pid/robot.c
)
target_include_directories(pid_oled.elf
    PRIVATE include /usr/local/include /usr/lib/avr/include
)
target_link_libraries(pid_oled.elf ${ZARDUINO_LIB})
