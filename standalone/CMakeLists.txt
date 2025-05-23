project(sr1xx-standalone)
cmake_minimum_required(VERSION 3.14)

include_directories(sr1xx-standalone ../src)
include_directories(sr1xx-standalone ${PROJECT_BINARY_DIR})

find_program(CARGO cargo)
 
if(NOT CARGO)
    message(FATAL_ERROR "Cargo not found! Please install Rust cargo.")
endif()

include(ExternalProject)

ExternalProject_Add(
    pdl_compiler
    DOWNLOAD_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND cargo build COMMAND cargo build --release
    BINARY_DIR "${CMAKE_SOURCE_DIR}/../pdl"
    INSTALL_COMMAND ""
    LOG_BUILD ON)

    add_custom_command(
        OUTPUT uci_packets.h
        COMMAND ${CMAKE_SOURCE_DIR}/../pdl/target/release/pdlc ${CMAKE_SOURCE_DIR}/../pica/src/uci_packets.pdl --output-format json >
                uci_packets.json && PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/../pdl/pdl-compiler/scripts
                ${CMAKE_SOURCE_DIR}/../src/scripts/generate_c_backend.py --input uci_packets.json --output uci_packets.h --prefix uci
        DEPENDS pdl_compiler
        DEPENDS ${CMAKE_SOURCE_DIR}/../pica/src/uci_packets.pdl
    )

add_custom_target(Generated DEPENDS uci_packets.h)

add_executable(sr1xx-standalone
    main.c
    uci_packets.h
    ../src/sr1xx_driver.c
    ../src/hal/sr1xx-core.c
    ../src/hal/sr1xx-session.c
    ../src/hal/sr1xx-range.c
    ../src/hal/sr1xx-thread.c
    ../src/hal/sr1xx.c
    ../src/hal/uci_protocol.c
    ../src/hal/linux/sr1xx-dev.c
    ../src/hal/linux/sr1xx-fwd.c
    ../src/hal/linux/parameters.c
)