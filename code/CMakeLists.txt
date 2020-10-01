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

add_executable(state_space_params.elf
    src/common/buffer.c
    src/common/interface_common.c
    src/common/interface_param.c
    src/common/main.c
    src/common/motors.c
    src/state_space/params.c
)
target_include_directories(state_space_params.elf
    PRIVATE include /usr/local/include /usr/lib/avr/include
)
target_link_libraries(state_space_params.elf ${ZARDUINO_LIB})

add_executable(state_space_radio.elf
    src/common/buffer.c
    src/common/interface_common.c
    src/common/interface_radio.c
    src/common/main.c
    src/common/motors.c
    src/state_space/radio.c
)
target_include_directories(state_space_radio.elf
    PRIVATE include /usr/local/include /usr/lib/avr/include
)
target_link_libraries(state_space_radio.elf ${ZARDUINO_LIB})

add_executable(observer_params.elf
    src/common/buffer.c
    src/common/interface_common.c
    src/common/interface_param.c
    src/common/main.c
    src/common/motors.c
    src/observer/params.c
)
target_include_directories(observer_params.elf
    PRIVATE include /usr/local/include /usr/lib/avr/include
)
target_link_libraries(observer_params.elf ${ZARDUINO_LIB})

add_executable(test_transient.elf
    src/common/buffer.c
    src/common/interface_common.c
    src/common/interface_uart.c
    src/common/main.c
    src/common/motors.c
    src/test/transient.c
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
    src/test/oscillatory.c
)
target_include_directories(test_oscillatory.elf
    PRIVATE include /usr/local/include /usr/lib/avr/include
)
target_link_libraries(test_oscillatory.elf ${ZARDUINO_LIB})

add_executable(test_constant.elf
    src/common/buffer.c
    src/common/interface_common.c
    src/common/interface_uart.c
    src/common/main.c
    src/common/motors.c
    src/test/constant.c
)
target_include_directories(test_constant.elf
    PRIVATE include /usr/local/include /usr/lib/avr/include
)
target_link_libraries(test_constant.elf ${ZARDUINO_LIB})
