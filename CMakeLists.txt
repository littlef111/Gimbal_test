#此文件从模板自动生成! 请勿更改!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.31)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(2025Gimbal C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

include_directories(Core/Inc USB_DEVICE/App USB_DEVICE/Target Drivers/STM32F4xx_HAL_Driver/Inc Drivers/STM32F4xx_HAL_Driver/Inc/Legacy Middlewares/Third_Party/FreeRTOS/Source/include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F Middlewares/ST/STM32_USB_Device_Library/Core/Inc Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc Drivers/CMSIS/Device/ST/STM32F4xx/Include Drivers/CMSIS/Include Middlewares/ST/ARM/DSP/Inc)

# User's files include
include_directories(
        User
        User/Framework

        User/Framework/DEBUGC
        User/Framework/GIMBALC
        User/Framework/IMU
        User/Framework/IMU/algorithm
        User/Framework/IMU/algorithm/Include
        User/Framework/IMU/controller
        User/Framework/IMU/device
        User/Framework/IWDG
        User/Framework/LED
        User/Framework/PID
        User/Framework/REMOTEC
        User/Framework/SHOOTC
        User/Framework/SMC
        User/Framework/VISION

        User/MCUDriver
        User/MCUDriver/CAN
        User/MCUDriver/DELAY
        User/MCUDriver/IMU
        User/MCUDriver/LED
        User/MCUDriver/REMOTEC
        User/MCUDriver/SPI
        User/MCUDriver/TIM5
        User/MCUDriver/IT

        User/Tasks
        User/Tasks/IMU_Task
        User/Tasks/GIMBAL_Task
        User/Tasks/VISIONCOM_Task

        User/Includes/PID_stm32
)

# global macro definition
add_definitions(
        -DDEBUG
        -DUSE_HAL_DRIVER
        -DSTM32F407xx
######## User's definition ########
        -DARM_MATH_CM4
        -D__FPU_USED=1U
        -D__FPU_PRESENT=1U
        -DARM_MATH_MATRIX_CHECK
        -DARM_MATH_ROUNDING
)

file(GLOB_RECURSE SOURCES "Core/*.*" "Middlewares/*.*" "USB_DEVICE/*.*" "Drivers/*.*"
######## User's sources ########
        "User/*.*"
        "User/Includes/PID_stm32/*.*"
)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F407IGHX_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

link_directories(${PROJECT_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib)
link_libraries("arm_cortexM4lf_math.a")

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

# target link ahrs.lib
target_link_libraries(${PROJECT_NAME}.elf ${PROJECT_SOURCE_DIR}/User/Framework/IMU/algorithm/AHRS.lib)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
