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

add_executable(controller_1.elf
    src/common/buffer.c
    src/common/interface_common.c
    src/common/interface_param.c
    src/common/main.c
    src/common/motors.c
    src/robot/controller_1.c
)
target_include_directories(controller_1.elf
    PRIVATE include /usr/local/include /usr/lib/avr/include
)
target_link_libraries(controller_1.elf ${ZARDUINO_LIB})

add_executable(controller_2.elf
    src/common/buffer.c
    src/common/interface_common.c
    src/common/interface_param.c
    src/common/main.c
    src/common/motors.c
    src/robot/controller_2.c
)
target_include_directories(controller_2.elf
    PRIVATE include /usr/local/include /usr/lib/avr/include
)
target_link_libraries(controller_2.elf ${ZARDUINO_LIB})

add_executable(test_transient.elf
    src/common/buffer.c
    src/common/interface_common.c
    src/common/interface_uart.c
    src/common/main.c
    src/common/motors.c
    src/robot/test_transient.c
)
target_include_directories(test_transient.elf
    PRIVATE include /usr/local/include /usr/lib/avr/include
)
target_link_libraries(test_transient.elf ${ZARDUINO_LIB})

add_executable(test_oscillatory.elf
    src/common/buffer.c
    src/common/interface_common.c
    src/common/interface_param.c
    src/common/interface_uart.c
    src/common/main.c
    src/common/motors.c
    src/robot/test_oscillatory.c
)
target_include_directories(test_oscillatory.elf
    PRIVATE include /usr/local/include /usr/lib/avr/include
)
target_link_libraries(test_oscillatory.elf ${ZARDUINO_LIB})

add_executable(test_pwm.elf
    src/common/buffer.c
    src/common/interface_common.c
    src/common/interface_uart.c
    src/common/main.c
    src/common/motors.c
    src/robot/test_pwm.c
)
target_include_directories(test_pwm.elf
    PRIVATE include /usr/local/include /usr/lib/avr/include
)
target_link_libraries(test_pwm.elf ${ZARDUINO_LIB})
