rock_library(terep
    SOURCES problems/Scheduling.cpp
    HEADERS problems/Scheduling.hpp)

rock_executable(terep_bin Main.cpp
    DEPS terep)

