rock_library(terep
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(terep_bin Main.cpp
    DEPS terep)

