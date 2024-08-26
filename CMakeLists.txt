cmake_minimum_required(VERSION 3.16)
project(gorobot VERSION 0.1.0 LANGUAGES C CXX)

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(DLL_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin/win64d")
option(BUILD_TESTING "Build the testing tree." ON)

# Disable min and max macros on Windows 
add_definitions(-DNOMINMAX)

include(CTest)
enable_testing()

if(NOT DEFINED ENV{GOCATORROOT})
    message(FATAL_ERROR "La variable de entorno GOCATORROOT no está definida")
endif()

## GOCATOR SDK
include_directories(
    $ENV{GOCATORROOT}/Gocator/GoSdk
    $ENV{GOCATORROOT}/Platform/kApi
    )
    
link_directories(
    $ENV{GOCATORROOT}/lib/win64d
    lib/win64d
    )

add_library(dome_gen_lib
    src/DomePoseGenerator.cpp
)

target_include_directories(dome_gen_lib PRIVATE 
    src/include
)

target_link_libraries(dome_gen_lib PRIVATE
    kApi
    GoSdk
    GoRobot
    Eigen3::Eigen
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
    Eigen3::Eigen
    dome_gen_lib
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

add_subdirectory(tests)