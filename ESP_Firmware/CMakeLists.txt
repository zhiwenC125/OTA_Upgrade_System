# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# MINIMAL_BUILD 已移除 — WiFi/MQTT 需要完整组件集
project(esp32_gateway)
