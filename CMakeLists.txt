# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")
set(picoSdk ${USERHOME}/.pico-sdk)
add_definitions(-DPICO_RP2040 -DFREE_RTOS_KERNEL_SMP -DLIB_PICO_MULTICORE=1)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(PiPico_Communication_Adapter C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(PiPico_Communication_Adapter main.cpp )

pico_set_program_name(PiPico_Communication_Adapter "PiPico_Communication_Adapter")
pico_set_program_version(PiPico_Communication_Adapter "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(PiPico_Communication_Adapter 0)
pico_enable_stdio_usb(PiPico_Communication_Adapter 1)

# Add the standard library to the build
target_link_libraries(PiPico_Communication_Adapter
        pico_stdlib hardware_uart)

# Add the standard include files to the build
target_include_directories(PiPico_Communication_Adapter PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        include/
        ${picoSdk}/sdk/2.1.1/src/rp2_common/hardware_irq/include
)

add_library(FreeRTOS STATIC
modbus/FreeRTOS-Kernel/croutine.c
modbus/FreeRTOS-Kernel/event_groups.c
modbus/FreeRTOS-Kernel/list.c
modbus/FreeRTOS-Kernel/queue.c
modbus/FreeRTOS-Kernel/stream_buffer.c
modbus/FreeRTOS-Kernel/tasks.c
modbus/FreeRTOS-Kernel/timers.c 
modbus/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/port.c
modbus/FreeRTOS-Kernel/portable/MemMang/heap_4.c 
modbus/src/FreeRTOS_Support_Files/IdleMemory.c)

target_include_directories(FreeRTOS PUBLIC
modbus/FreeRTOS-Kernel/include
include/
modbus/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/include
${picoSdk}/sdk/2.1.1/src/common/pico_base_headers/include
${picoSdk}/sdk/2.1.1/src/common/pico_sync/include
${picoSdk}/sdk/2.1.1/src/common/pico_time/include
${picoSdk}/sdk/2.1.1/src/rp2_common/pico_platform_compiler/include
${picoSdk}/sdk/2.1.1/src/rp2_common/pico_platform_sections/include
${picoSdk}/sdk/2.1.1/src/rp2_common/pico_platform_panic/include
${picoSdk}/sdk/2.1.1/src/rp2_common/hardware_sync/include
${picoSdk}/sdk/2.1.1/src/rp2_common/hardware_sync_spin_lock/include
${picoSdk}/sdk/2.1.1/src/rp2_common/hardware_base/include
${picoSdk}/sdk/2.1.1/src/rp2_common/hardware_clocks/include
${picoSdk}/sdk/2.1.1/src/rp2_common/hardware_exception/include
${picoSdk}/sdk/2.1.1/src/rp2_common/hardware_irq/include
${picoSdk}/sdk/2.1.1/src/rp2_common/hardware_timer/include
${picoSdk}/sdk/2.1.1/src/rp2_common/pico_multicore/include
${picoSdk}/sdk/2.1.1/src/rp2040/pico_platform/include
${picoSdk}/sdk/2.1.1/src/rp2040/hardware_regs/include
${picoSdk}/sdk/2.1.1/src/rp2040/hardware_structs/include
${CMAKE_BINARY_DIR}/generated/pico_base
modbus/MODBUS-LIB/Inc)

add_library(Modbus STATIC modbus/MODBUS-LIB/Src/Modbus.c)
target_link_libraries(Modbus pico_stdlib hardware_uart)
target_include_directories(Modbus PUBLIC
    modbus/FreeRTOS-Kernel/include
    include/
    modbus/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/include
    modbus/MODBUS-LIB/Inc)

pico_add_extra_outputs(PiPico_Communication_Adapter)

target_link_libraries(PiPico_Communication_Adapter hardware_exception pico_multicore FreeRTOS pico-cpp Modbus hardware_pio)
