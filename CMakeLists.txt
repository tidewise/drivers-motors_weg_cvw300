# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)
set(CMAKE_CXX_STANDARD 17)
rock_init(motors_weg_cvw300 0.1)
rock_standard_layout()
