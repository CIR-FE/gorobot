cmake_minimum_required(VERSION 3.16)
project(gorobot VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
option(BUILD_TESTING "Build the testing tree." ON)

# Disable min and max macros on Windows 
add_definitions(-DNOMINMAX)

if(NOT DEFINED ENV{GOCATORROOT})
    message(FATAL_ERROR "La variable de entorno GOCATORROOT no está definida")
endif()

# Define gocatorarch based on build type
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(GOCATORARCH "win64d")
else()
    set(GOCATORARCH "win64")
endif()

set(DLL_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
find_package(spdlog CONFIG REQUIRED)

## GOCATOR SDK
include_directories(
    $ENV{GOCATORROOT}/Gocator/GoSdk
    $ENV{GOCATORROOT}/Platform/kApi
    )
    
link_directories(
    $ENV{GOCATORROOT}/lib/${GOCATORARCH}
    lib
    )

add_executable(gorobot 
    src/KukaRobotDriver.cpp
    src/KukaLaserLine_EyeOnHand.cpp
    )


target_include_directories(gorobot PRIVATE 
    include
    )

target_link_libraries(gorobot PRIVATE
    kApi
    GoSdk
    GoRobot
    spdlog::spdlog
    )

# Copy DLLs
add_custom_command(TARGET gorobot POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $ENV{GOCATORROOT}/bin/${GOCATORARCH}/kApi.dll
    $<TARGET_FILE_DIR:gorobot>
    )

add_custom_command(TARGET gorobot POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $ENV{GOCATORROOT}/bin/${GOCATORARCH}/GoSdk.dll
    $<TARGET_FILE_DIR:gorobot>
    )

add_custom_command(TARGET gorobot POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${DLL_DIRECTORY}/GoRobot.dll
    $<TARGET_FILE_DIR:gorobot>)