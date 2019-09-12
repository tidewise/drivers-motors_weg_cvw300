rock_library(motors_weg_cvw300
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(motors_weg_cvw300_bin Main.cpp
    DEPS motors_weg_cvw300)

