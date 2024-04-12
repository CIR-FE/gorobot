find_package(GTest CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

# Add test cpp file
add_executable(matrix_mul_test
    matrix_mul_test.cc
)

include_directories(
    ${CMAKE_SOURCE_DIR}/src/include
)

target_link_libraries(matrix_mul_test 
    kApi
    GoSdk
    GoRobot
    Eigen3::Eigen
    GTest::gtest 
    GTest::gtest_main)

# Copy DLLs
add_custom_command(TARGET matrix_mul_test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $ENV{GO_SDK_4}/bin/win64d/kApi.dll
    $<TARGET_FILE_DIR:matrix_mul_test>
)

add_custom_command(TARGET matrix_mul_test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $ENV{GO_SDK_4}/bin/win64d/GoSdk.dll
    $<TARGET_FILE_DIR:matrix_mul_test>
)

add_custom_command(TARGET matrix_mul_test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${DLL_DIRECTORY}/GoRobot.dll
    $<TARGET_FILE_DIR:matrix_mul_test>
)

add_test(matrix_mul_test matrix_mul_test)