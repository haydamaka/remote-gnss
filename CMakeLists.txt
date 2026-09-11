cmake_minimum_required(VERSION 3.22)

if(NOT DEFINED ENV{IDF_PATH})
    message(FATAL_ERROR "IDF_PATH is not set. Source esp-idf/export.sh, or set it as the CLion toolchain environment file.")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(remote-gnss)
