cmake_minimum_required(VERSION 3.8)
project(nxp-uwb-ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
# Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()


find_program(CARGO cargo)

if(NOT CARGO)
    message(FATAL_ERROR "Cargo not found! Please install Rust cargo.")
endif()

include(ExternalProject)

ExternalProject_Add(
    pdl_compiler
    DOWNLOAD_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND cargo build
    COMMAND cargo build --release
    BINARY_DIR "${CMAKE_SOURCE_DIR}/pdl"
    INSTALL_COMMAND ""
    LOG_BUILD ON)

add_custom_command(
    OUTPUT uci_packets.h
    COMMAND ${CMAKE_SOURCE_DIR}/pdl/target/release/pdlc ${CMAKE_SOURCE_DIR}/pica/src/uci_packets.pdl --output-format json >
            uci_packets.json && PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/pdl/pdl-compiler/scripts
            ${CMAKE_SOURCE_DIR}/src/scripts/generate_c_backend.py --input uci_packets.json --output uci_packets.h --prefix uci
    DEPENDS pdl_compiler
    DEPENDS ${CMAKE_SOURCE_DIR}/pica/src/uci_packets.pdl
)

add_custom_target(Generated DEPENDS uci_packets.h)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uwb_msgs REQUIRED)

add_executable(node
               uci_packets.h
               src/sr1xx_node.cpp
               src/sr1xx_driver.c
               src/hal/sr1xx-core.c
               src/hal/sr1xx-session.c
               src/hal/sr1xx-range.c
               src/hal/sr1xx-thread.c
               src/hal/sr1xx.c
               src/hal/uci_protocol.c
               src/hal/linux/sr1xx-dev.c
               src/hal/linux/sr1xx-fwd.c
               src/hal/linux/parameters.c)

include_directories(node src)
include_directories(node ${PROJECT_BINARY_DIR})


add_definitions(-w)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_target_dependencies(node rclcpp std_msgs uwb_msgs)

install(TARGETS
  node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/twr_point_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
