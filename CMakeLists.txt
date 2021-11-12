cmake_minimum_required(VERSION 3.10)

# STM32L0, STM32L011, STM32L011F3

project(capn-blinky C CXX ASM)

set(CC_FLAGS
    -mcpu=cortex-m0plus
    -Wnull-dereference
    -mthumb
    -mfloat-abi=soft
    -fdata-sections
    -ffunction-sections
    -flto
    -ffat-lto-objects
    -Wall
    -Wno-strict-aliasing 
    -Wno-format
    -ffast-math)

set(CXX_FLAGS 
    -Wall 
    -Wextra
    -Wdouble-promotion
    -Wshadow 
    -Wfloat-conversion 
    -Wno-volatile
    -Wformat=2 
    -std=c++20 
    -fno-rtti 
    -fno-exceptions)

set(LD_FLAGS
    -T${CMAKE_SOURCE_DIR}/STM32CubeIDE/STM32L011F3UX_FLASH.ld
    -flto
    -ffat-lto-objects
    -mcpu=cortex-m4
    -mthumb
    -mfloat-abi=soft
    -specs=nosys.specs
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--gc-sections)

set(DEBUG_FLAGS
    -Og)

set(RELEASE_FLAGS   
    -Os)

set(DEFINITIONS
    USE_HAL_DRIVER 
    CORE_CM0PLUS 
    STM32L011xx)

set(BASE_ADDRESS 0x08000000)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
set(DUMP_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.S)

include_directories(.)
include_directories(STM32CubeIDE/Core/Inc)
include_directories(STM32CubeIDE/Drivers/CMSIS/Include)
include_directories(STM32CubeIDE/Drivers/CMSIS/Device/ST/STM32L0xx/Include)
include_directories(STM32CubeIDE/Drivers/STM32L0xx_HAL_Driver/Inc)

file(GLOB_RECURSE STM32CUBEIDE_SRC ${PROJECT_SOURCE_DIR}/STM32CubeIDE/Core/Src/*.c)
file(GLOB_RECURSE STM32CUBEIDE_SRC ${PROJECT_SOURCE_DIR}/STM32CubeIDE/Drivers/STM32L0xx_HAL_Driver/Src/*.c)
file(GLOB_RECURSE STM32CUBEIDE_SRC ${PROJECT_SOURCE_DIR}/STM32CubeIDE/*.c)

add_executable(${PROJECT_NAME}.elf 
    ${PROJECT_SOURCE_DIR}/capn-blinky.cpp
    ${PROJECT_SOURCE_DIR}/STM32CubeIDE/Core/Startup/startup_stm32l011f3ux.s
    ${STM32CUBEIDE_SRC})

# Generate version.h
find_package(Git)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    execute_process(COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} OUTPUT_VARIABLE GIT_SHORT_SHA OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(COMMAND ${GIT_EXECUTABLE} rev-list HEAD --count WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} OUTPUT_VARIABLE GIT_REV_COUNT OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(COMMAND ${GIT_EXECUTABLE} show -s --format=%ad HEAD --date=iso-strict WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} OUTPUT_VARIABLE GIT_COMMIT_DATE OUTPUT_STRIP_TRAILING_WHITESPACE)
else()
    set(GIT_SHORT_SHA "unknown")
    set(GIT_REV_COUNT "unknown")
    set(GIT_COMMIT_DATE "unknown")
endif()

include_directories(${CMAKE_BINARY_DIR})
configure_file("${PROJECT_SOURCE_DIR}/version.h.in" "${CMAKE_BINARY_DIR}/version.h" @ONLY)

include(./cmake/utils.cmake)

if("${CMAKE_C_COMPILER_ID}" STREQUAL "Clang")
    include(./cmake/clang_utils.cmake)
    clang_utils_get_arm_gcc_version(${ARM_GCC_C_COMPILER})
    clang_utils_get_arm_gcc_sysroot(${ARM_GCC_C_COMPILER} ${CC_FLAGS})
    clang_utils_get_arm_gcc_multilib_dir(${ARM_GCC_C_COMPILER} ${CC_FLAGS})
    clang_utils_get_arm_gcc_lib_dir(${ARM_GCC_C_COMPILER} ${CC_FLAGS})
    clang_utils_get_arm_gcc_crt(${ARM_GCC_SYSROOT} ${ARM_GCC_MULTIDIR} ${ARM_GCC_LIBGCC_DIR})
    string(APPEND CMAKE_EXE_LINKER_FLAGS" -nostdlib -L${ARM_GCC_SYSROOT}/lib/${ARM_GCC_MULTIDIR} -L${ARM_GCC_LIBGCC_DIR}")
    string(APPEND CMAKE_EXE_LINKER_FLAGS " -lgcc -lnosys -lc")
	include_directories(${ARM_GCC_SYSROOT}/include/c++/${ARM_GCC_VERSION})
	include_directories(${ARM_GCC_SYSROOT}/include/c++/${ARM_GCC_VERSION}/arm-none-eabi/${ARM_GCC_MULTIDIR})
	if(ARM_GCC_VERSION LESS "10.2")
		message(FATAL_ERROR "GCC must be version 10.2 or higher.")
	endif(ARM_GCC_VERSION LESS "10.2")
else()
	if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		message(FATAL_ERROR "Compiler must be GCC.")
	endif(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	if(CMAKE_CXX_COMPILER_VERSION LESS "10.2")
		message(FATAL_ERROR "GCC must be version 10.2 or higher.")
	endif(CMAKE_CXX_COMPILER_VERSION LESS "10.2")
endif()

target_compile_definitions(${PROJECT_NAME}.elf PRIVATE ${DEFINITIONS})
target_compile_options(${PROJECT_NAME}.elf PRIVATE ${CC_FLAGS})
target_link_options(${PROJECT_NAME}.elf PRIVATE ${LD_FLAGS})
target_compile_options(${PROJECT_NAME}.elf PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:${CXX_FLAGS}>")
target_compile_options(${PROJECT_NAME}.elf PRIVATE "$<$<CONFIG:DEBUG>:${DEBUG_FLAGS}>")
target_compile_options(${PROJECT_NAME}.elf PRIVATE "$<$<CONFIG:RELEASE>:${RELEASE_FLAGS}>")

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJDUMP} -D $<TARGET_FILE:${PROJECT_NAME}.elf> > ${DUMP_FILE}
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
    COMMAND ${CMAKE_OBJCOPY} -O ihex -R .eeprom -R .fuse -R .lock -R .signature $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
    COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}")
    
set(PROGRAM_CMD "./openocd -f stlink.cfg -f stm32wlx.cfg -c \"program ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin ${BASE_ADDRESS} verify reset exit\"")
install(CODE "execute_process(COMMAND ${PROGRAM_CMD} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/openocd)")
