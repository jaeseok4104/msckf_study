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
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTEST=1")
    enable_testing()
endif()

# if((run or RUN) and (tracker and TRACKER))
#     add_subdirectory(run)
# endif()