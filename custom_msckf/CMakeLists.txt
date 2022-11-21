cmake_minimum_required(VERSION 3.23.0)
project(custom_msckf)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -O3 -w -fPIC")
file(GLOB_RECURSE src_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)
file(GLOB_RECURSE header_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/*.h)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# find_package(OpenCV 4
#     NO_MODULE
#     PATHS /usr/local
#     NO_DEFAULT_PATH
# )

add_library(
    ${PROJECT_NAME}
    ${src_files}
    ${header_files}
)

target_link_libraries(
    ${PROJECT_NAME}
    Eigen3::Eigen
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

if (test OR TEST)
    enable_testing()
    file(GLOB_RECURSE test_sources RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/test test/*_test.cpp)

    foreach(test_source ${test_sources})
        string(REPLACE ".cpp" "" test_name ${test_source})
        set(test_binary_name ${PROJECT_NAME}_${test_name})

        add_executable(${test_binary_name} test/${test_source})
        target_link_libraries(${test_binary_name} gtest_main gtest glog)
        target_link_libraries(${test_binary_name} ${PROJECT_NAME})
        add_test(NAME ${test_binary_name} COMMAND ${test_binary_name})
    endforeach()
endif()