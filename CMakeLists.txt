cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(physical_monitor)

target_sources(app PRIVATE src/main_ali.c)
