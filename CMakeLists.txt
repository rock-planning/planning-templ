# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
set(CMAKE_COMPILE_EXPORT_COMMANDS on)
find_package(Rock)
add_definitions("-std=c++11")
rock_init(templ 0.1)
rock_standard_layout()
