# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(fpga_loader)

target_sources(app PRIVATE src/main.c)
