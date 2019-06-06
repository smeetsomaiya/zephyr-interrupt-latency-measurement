# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(measure_80)

target_sources(app PRIVATE src/main.c)
#zephyr_library_include_directories($ENV{ZEPHYR_BASE}/include/posix)
#add_executable(clock clock.c)
#install(TARGETS clock DESTINATION bin)
#find_package( RT        REQUIRED )

