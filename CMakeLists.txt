# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
#set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)


# PROJECT Version should be major.minor.patch - no more than 10 bits each!
set(PROJECT_VER "0.0.9")

add_compile_definitions(SSID_PASSWORD="$ENV{SSID_PASSWORD}")
add_compile_definitions(SSID="$ENV{SSID}")
add_compile_definitions(SSID_PASSWORD2="$ENV{SSID_PASSWORD2}")
add_compile_definitions(SSID2="$ENV{SSID2}")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(heatpump)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "main/client.crt" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "main/client.key" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "main/CA.crt" TEXT)
