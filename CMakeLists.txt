# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
set(CMAKE_COMPILE_EXPORT_COMMANDS on)
set(ROCK_TEST_ENABLED on)
find_package(Rock)
rock_activate_cxx11()
rock_init(templ 0.1)
rock_find_qt4()
rock_standard_layout()
