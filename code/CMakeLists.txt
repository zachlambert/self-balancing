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

add_executable(pid.elf
    src/core/buffer.c
    src/core/interface.c
    src/core/main.c
    src/core/motors.c
    src/core/robot.c
    src/control/pid_control.c
)
target_include_directories(pid.elf
    PRIVATE include /usr/local/include /usr/lib/avr/include
)
target_link_libraries(pid.elf ${ZARDUINO_LIB})

add_executable(pid_effort.elf
    src/core/buffer.c
    src/core/interface.c
    src/core/main.c
    src/core/motors.c
    src/core/robot.c
    src/control/pid_effort_control.c
)
target_include_directories(pid_effort.elf
    PRIVATE include /usr/local/include /usr/lib/avr/include
)
target_link_libraries(pid_effort.elf ${ZARDUINO_LIB})
