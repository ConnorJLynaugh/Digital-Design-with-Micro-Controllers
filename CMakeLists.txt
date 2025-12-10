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
set(PICO_BOARD pico_w CACHE STRING "Board type")

# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(QUADRAPED_WIFI C CXX ASM)

# initialize the sdk
pico_sdk_init()

add_executable(QUADRAPED_WIFI
    # Robot
    main.c
    movement_library.c
    pca9685.c
    tfluna_i2c.c
    mlx90614.c
    mpu6050.c
    mapping.c
    # Wifi
    wifi.c
    dhcpserver/dhcpserver.c
    dnsserver/dnsserver.c
)

target_include_directories(QUADRAPED_WIFI PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/dhcpserver
    ${CMAKE_CURRENT_LIST_DIR}/dnsserver
)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(QUADRAPED_WIFI 
    pico_stdlib 
    hardware_i2c
    pico_bootsel_via_double_reset

    # Wifi
    pico_cyw43_arch_lwip_threadsafe_background
    #pico_cyw43_arch_lwip_poll
)

# You can change the address below to change the address of the access point
pico_configure_ip4_address(QUADRAPED_WIFI PRIVATE
        CYW43_DEFAULT_IP_AP_ADDRESS 192.168.4.1
)

pico_add_extra_outputs(QUADRAPED_WIFI)




