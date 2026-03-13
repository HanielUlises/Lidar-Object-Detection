cmake_minimum_required(VERSION 3.16)

project(playback)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Remove problematic vtk library if present
list(REMOVE_ITEM PCL_LIBRARIES vtkproj4)

add_executable(quizRansac
    ransac2d.cpp
    ../../render/render.cpp
)

target_link_libraries(quizRansac
    ${PCL_LIBRARIES}
)