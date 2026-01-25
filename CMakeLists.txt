cmake_minimum_required(VERSION 3.16)

project(playback
    VERSION 1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PCL REQUIRED COMPONENTS
    common
    io
    visualization
    filters
    segmentation
    kdtree
    search
)

add_executable(environment
    src/environment.cpp
    src/render/render.cpp
    src/processPointClouds.cpp
)

target_include_directories(environment PRIVATE
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(environment PRIVATE
    ${PCL_LIBRARIES}
)

target_compile_definitions(environment PRIVATE
    ${PCL_DEFINITIONS}
)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
