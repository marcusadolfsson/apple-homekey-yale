cmake_minimum_required(VERSION 3.16)
# Reuse only the components this app needs from the firmware in the parent
# directory. Pointing at ../components wholesale would pull in HomeSpan (and
# with it Arduino), which the doorbell deliberately does not use.
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../components/pn532_cxx"
    "${CMAKE_CURRENT_LIST_DIR}/../components/loggable"
    "${CMAKE_CURRENT_LIST_DIR}/../components/loggable_espidf")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(relay-doorbell)
