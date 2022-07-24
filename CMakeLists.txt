# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(transportino_board C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

# Add any PICO_CONFIG entries specified in the Advanced settings
add_compile_definitions(PICO_DEFAULT_I2C=0)
add_compile_definitions(PICO_DEFAULT_I2C_SCL_PIN=1)
add_compile_definitions(PICO_DEFAULT_I2C_SDA_PIN=0)
add_compile_definitions(PICO_DEFAULT_LED_PIN=7)

include_directories(./include)
file(GLOB_RECURSE FILES src/*.c)
add_executable(transportino_board ${FILES})

pico_set_program_name(transportino_board "transportino_board")
pico_set_program_version(transportino_board "0.1")

pico_enable_stdio_uart(transportino_board 0)
pico_enable_stdio_usb(transportino_board 1)

add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

#Dev mode
#add_compile_definitions(DEV)

# Add the standard library to the build
target_link_libraries(transportino_board pico_stdlib)

target_include_directories(transportino_board PUBLIC ${CMAKE_SOURCE_DIR}/libs/icm20689/include)
target_link_libraries(transportino_board ${CMAKE_SOURCE_DIR}/libs/icm20689/libicm20689pico.a)

set(MICRO_ROS "${CMAKE_SOURCE_DIR}/../../micro_ros/sdk/libmicroros")

target_include_directories(transportino_board PUBLIC ${MICRO_ROS}/include)
target_link_libraries(transportino_board ${MICRO_ROS}/libmicroros.a)


# Add any user requested libraries from pico sdk
target_link_libraries(transportino_board
        hardware_i2c
        hardware_pwm
        hardware_watchdog
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")


pico_add_extra_outputs(transportino_board)

