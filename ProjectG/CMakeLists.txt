cmake_minimum_required(VERSION 3.25)
project(untitled4)

set(CMAKE_CXX_STANDARD 26)

add_executable(ProjectG main.cpp)

target_link_libraries(ProjectG OpenGL32 freeglut glu32)
