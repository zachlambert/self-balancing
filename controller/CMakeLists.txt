cmake_minimum_required(VERSION 3.0)
project(
    "Self Balancing Robot Controller"
    VERSION 0.1.0
    DESCRIPTION "The controller for my self-balancing robot project"
    LANGUAGES C
)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(tools/target_cmake_scripts/atmega328p.cmake)

add_executable(main.elf
    src/main.c
)

find_library(ZARDUINO_LIB
    NAMES zarduino
    PATHS /usr/local/lib
)
target_include_directories(main.elf
    PRIVATE include /usr/local/include /usr/lib/avr/include
)
target_link_libraries(main.elf ${ZARDUINO_LIB})
