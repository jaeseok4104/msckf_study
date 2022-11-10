cmake_minimum_required(VERSION 3.23.0)
project(msckf_study)

file(GLOB_RECURSE src_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)
file(GLOB_RECURSE header_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/*.h)

add_subdirectory(custom_msckf)

if(tracker OR TRACKER)
    find_package(Eigen3 3.3 REQUIRED NO_MODULE)
    find_package(OpenCV 4
        NO_MODULE
        PATHS /usr/local
        NO_DEFAULT_PATH
    )

    add_library(${project_name}_lib
        ${src_files}
        ${header_files}
    )

    target_include_directories(${project_name}_lib
        ${CMAKE_CURRENT_BINARY_DIR}
    )
    target_link_libraries(${project_name}_lib
        custom_msckf
    )
endif()

if (test OR TEST)
    enable_testing()
    get_filename_component(folder_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
    file(GLOB_RECURSE test_sources RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/test test/*_test.cpp)

    foreach(test_source ${test_sources})
        string(REPLACE ".cpp" "" test_name ${test_source})
        set(test_binary_name ${folder_name}_${test_name})

        add_executable(${test_binary_name} test/${test_source})
        target_link_libraries(${test_binary_name} GTest::GTest GTest::Main glog)
        target_link_libraries(${test_binary_name} pose_estimator_api)
        set_target_properties(${test_binary_name}
                PROPERTIES CXX_CLANG_TIDY ${CLANG_TIDY_BINARY_NAME}
                )
        add_test(NAME ${test_binary_name} COMMAND ${test_binary_name})
        gtest_discover_tests(${test_binary_name})
    endforeach()
endif()

# if((run or RUN) and (tracker and TRACKER))
#     add_subdirectory(run)
# endif()