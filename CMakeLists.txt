cmake_minimum_required(VERSION 3.22)

# 本工程直接使用电脑上 STM32CubeIDE 自带的 GNU Arm 编译器，不依赖 Keil。
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

if (WIN32)
    # VS/VS Code：优先使用工程内由 tools/setup-tools.ps1 解压的工具链；
    # 也可通过环境变量 ARM_GNU_TOOLCHAIN 指向任意 Arm GNU 工具链根目录。
    set(_PROJECT_ARM_GNU_TOOLS_DIR
        "${CMAKE_SOURCE_DIR}/.tools/arm-gnu-toolchain-14.3.rel1-mingw-w64-x86_64-arm-none-eabi/bin")
    if (DEFINED ENV{ARM_GNU_TOOLCHAIN})
        set(_DEFAULT_ARM_GNU_TOOLS_DIR "$ENV{ARM_GNU_TOOLCHAIN}/bin")
    elseif (EXISTS "${_PROJECT_ARM_GNU_TOOLS_DIR}/arm-none-eabi-gcc.exe")
        set(_DEFAULT_ARM_GNU_TOOLS_DIR "${_PROJECT_ARM_GNU_TOOLS_DIR}")
    else ()
        set(_DEFAULT_ARM_GNU_TOOLS_DIR "${_PROJECT_ARM_GNU_TOOLS_DIR}")
    endif ()
else ()
    set(_DEFAULT_ARM_GNU_TOOLS_DIR
        "/opt/st/stm32cubeide_2.2.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.linux64_1.0.100.202602081740/tools/bin")
endif ()

set(STM32_GNU_TOOLS_DIR "${_DEFAULT_ARM_GNU_TOOLS_DIR}"
    CACHE PATH "GNU Arm Embedded 工具链的 bin 目录")

if (WIN32)
    set(_ARM_EXE_SUFFIX ".exe")
else ()
    set(_ARM_EXE_SUFFIX "")
endif ()

if (NOT EXISTS "${STM32_GNU_TOOLS_DIR}/arm-none-eabi-gcc${_ARM_EXE_SUFFIX}")
    message(FATAL_ERROR
        "未找到 arm-none-eabi-gcc。请执行 tools/setup-tools.ps1，或设置 ARM_GNU_TOOLCHAIN 环境变量。")
endif ()
set(CMAKE_C_COMPILER   "${STM32_GNU_TOOLS_DIR}/arm-none-eabi-gcc${_ARM_EXE_SUFFIX}")
set(CMAKE_ASM_COMPILER "${STM32_GNU_TOOLS_DIR}/arm-none-eabi-gcc${_ARM_EXE_SUFFIX}")
set(CMAKE_OBJCOPY      "${STM32_GNU_TOOLS_DIR}/arm-none-eabi-objcopy${_ARM_EXE_SUFFIX}")
set(CMAKE_SIZE         "${STM32_GNU_TOOLS_DIR}/arm-none-eabi-size${_ARM_EXE_SUFFIX}")

project(machine_debug_DM_MC02_length_M2006 C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CPU_FLAGS
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard)

add_compile_options(
    ${CPU_FLAGS}
    -ffunction-sections
    -fdata-sections
    -fno-common
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-unused-variable
    -Wno-unused-function
    $<$<CONFIG:Debug>:-Og>
    $<$<CONFIG:Debug>:-g3>
    $<$<CONFIG:Release>:-O2>)

add_compile_definitions(
    USE_HAL_DRIVER
    STM32H723xx
    ARM_MATH_CM7
    __USE_RTOS)

set(FREERTOS_PORT
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1)

include_directories(
    Core/Inc
    USB_DEVICE/App
    USB_DEVICE/Target
    Drivers/STM32H7xx_HAL_Driver/Inc
    Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32H7xx/Include
    Drivers/CMSIS/Include
    Drivers/CMSIS/DSP/Include
    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ${FREERTOS_PORT}
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    App/inc
    Bsp/inc
    Algorithm/inc
    MDK-ARM
    Robot/Inc)

file(GLOB HAL_SOURCES CONFIGURE_DEPENDS
    Drivers/STM32H7xx_HAL_Driver/Src/*.c)

set(SOURCES
    # ST 官方 GNU 启动文件；MDK-ARM 中仍保留 Keil 版本。
    Core/startup_stm32h723xx_gcc.s

    # STM32H723 / MC02 底层。
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/freertos.c
    Core/Src/adc.c
    Core/Src/dma.c
    Core/Src/fdcan.c
    Core/Src/octospi.c
    Core/Src/spi.c
    Core/Src/tim.c
    Core/Src/usart.c
    Core/Src/stm32h7xx_it.c
    Core/Src/stm32h7xx_hal_msp.c
    Core/Src/stm32h7xx_hal_timebase_tim.c
    Core/Src/system_stm32h7xx.c
    Core/Src/syscalls.c

    # Type-C 原生 USB CDC。
    USB_DEVICE/App/usb_device.c
    USB_DEVICE/App/usbd_desc.c
    USB_DEVICE/App/usbd_cdc_if.c
    USB_DEVICE/Target/usbd_conf.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c

    # FreeRTOS，Cortex-M7 GCC 端口已随工程保存，不引用电脑外部目录。
    Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    Middlewares/Third_Party/FreeRTOS/Source/list.c
    Middlewares/Third_Party/FreeRTOS/Source/queue.c
    Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    Middlewares/Third_Party/FreeRTOS/Source/timers.c
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ${FREERTOS_PORT}/port.c

    # 板载 BMI088 与姿态解算。
    Bsp/BMI088driver.c
    Bsp/BMI088Middleware.c
    Bsp/w25q64.c
    Algorithm/MahonyAHRS.c
    App/imu_task.c

    # 学长工程业务层：文件名、全局变量、类型和自动流程继续沿用。
    MDK-ARM/Auto.c
    MDK-ARM/PID_task.c
    MDK-ARM/chassis_test.c
    MDK-ARM/ramp.c
    MDK-ARM/task_init.c
    MDK-ARM/vision_usb.c
    MDK-ARM/mechanism_motor.c
    Robot/Src/CANDrive.c
    Robot/Src/Chassis.c
    Robot/Src/CRC.c
    Robot/Src/motor.c
    Robot/Src/PID.c
    Robot/Src/wheel.c

    ${HAL_SOURCES})

add_executable(${PROJECT_NAME}.elf ${SOURCES})

target_link_options(${PROJECT_NAME}.elf PRIVATE
    ${CPU_FLAGS}
    -T${CMAKE_SOURCE_DIR}/STM32H723VGTX_FLASH.ld
    -Wl,--gc-sections
    -Wl,-u,_printf_float
    -Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map
    --specs=nano.specs
    --specs=nosys.specs)

target_link_libraries(${PROJECT_NAME}.elf PRIVATE m)

option(STM32_AUTO_FLASH "Build 后通过调试器/SWD自动下载" OFF)

# 编译完成后同时生成 CubeProgrammer 可烧录的 hex/bin，并复制到 Firmware。
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${PROJECT_NAME}.elf>
            ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${PROJECT_NAME}.elf>
            ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_COMMAND} -E make_directory
            ${CMAKE_SOURCE_DIR}/Firmware
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex
            ${CMAKE_SOURCE_DIR}/Firmware/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin
            ${CMAKE_SOURCE_DIR}/Firmware/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:${PROJECT_NAME}.elf>
            ${CMAKE_SOURCE_DIR}/Firmware/${PROJECT_NAME}.elf
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
    VERBATIM)

# CLion 中选择 daplink_flash 目标后，点击构建即可先编译、再用
# DAPLink/CMSIS-DAP 烧录，然后校验并复位。
if (NOT WIN32)
    add_custom_target(daplink_flash
        COMMAND "${CMAKE_SOURCE_DIR}/tools/daplink-flash.sh"
                "$<TARGET_FILE:${PROJECT_NAME}.elf>"
        DEPENDS ${PROJECT_NAME}.elf
        USES_TERMINAL
        VERBATIM)
endif ()

if (STM32_AUTO_FLASH AND WIN32)
    add_custom_target(flash ALL
        DEPENDS ${PROJECT_NAME}.elf
        COMMAND powershell.exe -NoProfile -ExecutionPolicy Bypass
                -File "${CMAKE_SOURCE_DIR}/tools/flash-stlink.ps1"
                -Firmware "$<TARGET_FILE:${PROJECT_NAME}.elf>"
        VERBATIM)
endif ()
