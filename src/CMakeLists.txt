rock_library(motors_weg_cvw300
    SOURCES Driver.cpp
    HEADERS Driver.hpp InverterStatus.hpp InverterTemperatures.hpp Configuration.hpp
    CurrentState.hpp MotorRatings.hpp FaultState.hpp
    DEPS_PKGCONFIG base-types modbus)

rock_executable(motors_weg_cvw300_ctl Main.cpp
    DEPS motors_weg_cvw300)
install(PROGRAMS ${CMAKE_SOURCE_DIR}/bin/motors_weg_cvw300_stress_test.sh DESTINATION bin RENAME motors_weg_cvw300_stress_test)
