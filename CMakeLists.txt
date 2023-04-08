cmake_minimum_required(VERSION 3.7)

set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME "Geometry")
project(${PROJECT_NAME})


set(Eigen3_DIR "/Users/gtt/Build/eigen")
list(APPEND CMAKE_PREFIX_PATH ${Eigen3_DIR})
find_package(Eigen3)

get_target_property(Eigen3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)

message(STATUS ${Eigen3_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})


set(GLFW3_DIR "/Users/gtt/Build/glfw")
list(APPEND CMAKE_PREFIX_PATH ${GLFW3_DIR})
find_package(glfw3)
get_target_property(GLFW3_INCLUDE_DIRS glfw INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS ${GLFW3_INCLUDE_DIRS})
include_directories(${GLFW3_INCLUDE_DIRS})

set(GLAD_DIR "${CMAKE_SOURCE_DIR}/thirdparty/glad")
add_library(ext_glad ${GLAD_DIR}/src/glad.c)

set(GLAD_INCLUDE_DIRS ${GLAD_DIR}/include)
include_directories(${GLAD_INCLUDE_DIRS})


add_executable(${PROJECT_NAME}_0 main.cxx
    GeometryMesh.h
    GeometryMesh.cpp
)

target_link_libraries(${PROJECT_NAME}_0 ext_glad glfw ${LIBS})
