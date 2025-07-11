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
        Core/Src/sys/delay/delay.c
        Core/Src/sys/Config/config.c
        Core/Src/Hardware/step_motor/step_motor.c
        Core/Src/Hardware/step_motor/step_Track.c
        Core/Src/Hardware/Motor/control.c
        Core/Src/Hardware/Motor/filter.c
        Core/Src/Hardware/Motor/filter.h
        Core/Src/Hardware/Motor/motor.c
        Core/Src/Hardware/Screen/OLED_IIC/OLED.c
        Core/Src/Hardware/Screen/OLED_IIC/OLED.h
        Core/Src/Hardware/Screen/OLED_IIC/OLED_Font.h
        Core/Src/Conmunication/serial.c
        Core/Src/Hardware/IMU/MPU/MPU6050/mpu6050.c
        Core/Src/Hardware/IMU/MPU/MPU6050/mpu6050.h
        Core/Src/Hardware/IMU/MPU/DMP/dmpKey.h
        Core/Src/Hardware/IMU/MPU/DMP/dmpmap.h
        Core/Src/Hardware/IMU/MPU/DMP/inv_mpu.c
        Core/Src/Hardware/IMU/MPU/DMP/inv_mpu.h
        Core/Src/Hardware/IMU/MPU/DMP/inv_mpu_dmp_motion_driver.c
        Core/Src/Hardware/IMU/MPU/DMP/inv_mpu_dmp_motion_driver.h
        Core/Src/Hardware/Magnetic/QMC5883/qmc5883.c
        Core/Src/Hardware/Magnetic/QMC5883/qmc5883.h
        Core/Src/Hardware/use.c
        Core/Src/Hardware/use.h
        Core/Src/Conmunication/IIC_moni/iic_moni.c
        Core/Src/Conmunication/SPI_moni/spi_moni.c
        Core/Src/Conmunication/SPI_moni/spi_moni.h
        Core/Src/Hardware/Distance/Tof/Vl53l0x_tof.c
        Core/Src/Hardware/Distance/Tof/Vl53l0x_tof.h
        Core/Src/Hardware/IMU/wit/wit_elec.c
        Core/Src/Hardware/IMU/wit/wit_elec.h
        #        Core/Src/Hardware/Distance/Tof/core/inc/vl53l0x_api.h
#        Core/Src/Hardware/Distance/Tof/core/inc/vl53l0x_api_calibration.h
#        Core/Src/Hardware/Distance/Tof/core/inc/vl53l0x_api_core.h
#        Core/Src/Hardware/Distance/Tof/core/inc/vl53l0x_api_ranging.h
#        Core/Src/Hardware/Distance/Tof/core/inc/vl53l0x_api_strings.h
#        Core/Src/Hardware/Distance/Tof/core/inc/vl53l0x_def.h
#        Core/Src/Hardware/Distance/Tof/core/inc/vl53l0x_device.h
#        Core/Src/Hardware/Distance/Tof/core/inc/vl53l0x_interrupt_threshold_settings.h
#        Core/Src/Hardware/Distance/Tof/core/inc/vl53l0x_tuning.h
#        Core/Src/Hardware/Distance/Tof/core/src/vl53l0x_api.c
#        Core/Src/Hardware/Distance/Tof/core/src/vl53l0x_api_calibration.c
#        Core/Src/Hardware/Distance/Tof/core/src/vl53l0x_api_core.c
#        Core/Src/Hardware/Distance/Tof/core/src/vl53l0x_api_ranging.c
#        Core/Src/Hardware/Distance/Tof/core/src/vl53l0x_api_strings.c
#        Core/Src/Hardware/Distance/Tof/platform/inc/vl53l0x_i2c_platform.h
#        Core/Src/Hardware/Distance/Tof/platform/inc/vl53l0x_platform.h
#        Core/Src/Hardware/Distance/Tof/platform/inc/vl53l0x_platform_log.h
#        Core/Src/Hardware/Distance/Tof/platform/inc/vl53l0x_types.h
#        Core/Src/Hardware/Distance/Tof/platform/src/vl53l0x_i2c_platform.c
#        Core/Src/Hardware/Distance/Tof/platform/src/vl53l0x_i2c_win_serial_comms.c
#        Core/Src/Hardware/Distance/Tof/platform/src/vl53l0x_platform.c
#        Core/Src/Hardware/Distance/Tof/platform/src/vl53l0x_platform_log.c
)

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
