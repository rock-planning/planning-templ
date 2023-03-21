# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.10)
project(templ
    VERSION 0.1
    DESCRIPTION "TemPl - Planning for Reconfigurable Multi-Robot Systems"
)
set(CMAKE_COMPILE_EXPORT_COMMANDS on)
set(ROCK_TEST_ENABLED on)
find_package(Rock)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_definitions(-Wall) # -Wpedantic)
set(CMAKE_BUILD_TYPE Release)

if(COVERAGE)
    if(CMAKE_BUILD_TYPE MATCHES Debug)
        add_definitions(-fprofile-arcs -ftest-coverage)
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs -ftest-coverage -lgcov")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fprofile-arcs -ftest-coverage -lgcov")
        set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fprofile-arcs -ftest-coverage -lgcov")

        add_custom_target(coverage
          COMMAND lcov --directory . --capture --no-external --output-file cov.info
          COMMAND lcov --remove cov.info '/opt' '*/test/*' '/usr/*' '*/install/*' --output-file cov.info.cleaned
          COMMAND genhtml -o ./cov cov.info.cleaned
          COMMAND cmake -E remove cov.info cov.info.cleaned
          WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        )
    else()
        message(FATAL_ERROR "Code coverage only works in Debug versions" )
    endif()
endif()

rock_init()
rock_find_qt5(Core Gui Xml Widgets PrintSupport Svg)
rock_standard_layout()
