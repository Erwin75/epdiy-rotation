# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
# This references epd_driver
set(EXTRA_COMPONENT_DIRS "../../src/")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ble_receive_jpg)

# This is just to avoid that components/jpegdec includes Arduino.h
idf_build_set_property(COMPILE_OPTIONS "-D __LINUX__" APPEND)