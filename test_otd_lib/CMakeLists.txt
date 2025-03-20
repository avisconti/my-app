# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_world)

target_sources(app PRIVATE src/main.c)

# external library part
set(otd_lib_dir ${CMAKE_CURRENT_SOURCE_DIR}/otd_lib)

add_library(lib_otd STATIC IMPORTED GLOBAL)
set_target_properties(lib_otd PROPERTIES IMPORTED_LOCATION ${otd_lib_dir}/lib_otd.a)
add_library(lib_common_utils STATIC IMPORTED GLOBAL)
set_target_properties(lib_common_utils PROPERTIES IMPORTED_LOCATION ${otd_lib_dir}/lib_common_utils.a)

target_include_directories(app PRIVATE ${otd_lib_dir})

target_link_libraries(app PRIVATE lib_otd lib_common_utils)

