cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME test)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/Src/Hardware/Motor/control.c
        Core/Src/Hardware/Motor/control.h
        Core/Src/Hardware/Motor/filter.c
        Core/Src/Hardware/Motor/filter.h
        Core/Src/Hardware/Motor/motor.c
        Core/Src/Hardware/Motor/motor.h
        Middlewares/ST/ARM/DSP/Inc/arm_math.h
        Core/Src/Hardware/IMU/MPU9250/iic_moni.c
        Core/Src/Hardware/IMU/MPU9250/iic_moni.h
        Core/Src/Hardware/IMU/MPU9250/mpu9250.c
        Core/Src/Hardware/IMU/MPU9250/mpu9250.h
        Core/Src/sys/delay/delay.c
        Core/Src/sys/delay/delay.h
        Core/Src/Hardware/step_motor/step_motor.c
        Core/Src/Hardware/step_motor/step_motor.h
        Core/Src/Hardware/step_motor/step_Track.c
        Core/Src/Hardware/step_motor/step_Track.h
        Core/Src/sys/Config/config.c
        Core/Src/sys/Config/config.h
        Core/Src/Conmunication/serial.c
        Core/Src/Conmunication/serial.h)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
