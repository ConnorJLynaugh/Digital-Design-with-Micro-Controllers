# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(QUADRAPED C CXX ASM)

# initialize the sdk
pico_sdk_init()

add_executable(QUADRAPED)

target_sources(QUADRAPED PRIVATE 
    main.c
    movement_library.c
    pca9685.c
)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(QUADRAPED 
    pico_stdlib 
    hardware_i2c
    pico_bootsel_via_double_reset
)

# create map/bin/hex file etc.
pico_add_extra_outputs(QUADRAPED)