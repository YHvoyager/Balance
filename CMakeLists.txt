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
set(CMAKE_PROJECT_NAME Balance)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
    Core/Src/ESP01S.c
    Core/Inc/ESP01S.h
    Core/Src/APP/PID.c
    Core/Inc/APP/PID.h
    Core/Inc/APP/APP_MPU6050.h
    Core/Inc/task.h
    Core/Src/KEY.c
    Core/Inc/KEY.h
    Core/Src/APP/APP_Encoder.c
    Core/Inc/APP/APP_Encoder.h
    Core/Inc/test/Encoder_test.h
    Core/Src/test/Encoder_test.c
    Core/Inc/printf.h
    Core/Src/printf.c
    Core/Src/qmath.c
    Core/Inc/qmath.h
    Core/Src/APP/APP_PID.c
    Core/Inc/APP/APP_PID.h
    Core/Src/APP/APP_Bat.c
    Core/Inc/APP/APP_Bat.h
    Core/Src/APP/APP_PWM.c
    Core/Inc/APP/APP_PWM.h
    Core/Inc/APP/APP_Control.h)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/APP/APP_Motor_Control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/APP/APP_Control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/APP/APP_RemotePID.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/oled.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/font.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/APP/APP_MPU6050.c 
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/APP/Vofa.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/ESP01S.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/APP
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/test
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
