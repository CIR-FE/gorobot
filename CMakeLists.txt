cmake_minimum_required(VERSION 3.16)
project(gorobot VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(DLL_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin/win64d")

# Disable min and max macros on Windows 
add_definitions(-DNOMINMAX)

include(CTest)
enable_testing()

if(NOT DEFINED ENV{GO_SDK_4})
    message(FATAL_ERROR "La variable de entorno GO_SDK_4 no está definida")
endif()

## GOCATOR SDK
include_directories(
    $ENV{GO_SDK_4}/Gocator/GoSdk
    $ENV{GO_SDK_4}/Platform/kApi
    )
    
link_directories(
    $ENV{GO_SDK_4}/lib/win64d
    lib/win64d
    )

add_executable(gorobot 
    src/KukaRobotDriver.cpp
    src/KukaLaserLine_EyeOnHand.cpp
    )

target_include_directories(gorobot PRIVATE 
    src/include
    )

target_link_libraries(gorobot PRIVATE
    kApi
    GoSdk
    GoRobot
    )

# Copy DLLs
add_custom_command(TARGET gorobot POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $ENV{GO_SDK_4}/bin/win64d/kApi.dll
    $<TARGET_FILE_DIR:gorobot>
    )

add_custom_command(TARGET gorobot POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $ENV{GO_SDK_4}/bin/win64d/GoSdk.dll
    $<TARGET_FILE_DIR:gorobot>
    )

add_custom_command(TARGET gorobot POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${DLL_DIRECTORY}/GoRobot.dll
    $<TARGET_FILE_DIR:gorobot>)
