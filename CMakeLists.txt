# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.9)
set(CMAKE_COMPILE_EXPORT_COMMANDS on)
set(ROCK_TEST_ENABLED on)
find_package(Rock)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_definitions(-Wall) # -Wpedantic)
rock_init(templ
    VERSION 0.1
    DESCRIPTION "TemPl - Planning for Reconfigurable Multi-Robot Systems"
)
rock_find_qt4()
rock_standard_layout()
