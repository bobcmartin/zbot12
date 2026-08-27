

cmake_minimum_required(VERSION 3.20)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
message(STATUS ">> Zephyr found at: ${ZEPHYR_BASE}")
message(STATUS ">> Using Board: ${BOARD}")
project(zbot12)
target_sources(app PRIVATE src/main.c)

