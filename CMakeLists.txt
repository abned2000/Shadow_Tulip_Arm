cmake_minimum_required(VERSION 3.10)
project(RobotArmControl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)


find_package(OpenCV REQUIRED)

find_package(CURL REQUIRED)

# Agrega el ejecutable
add_executable(control control.cpp motor.cpp)

add_executable(test_move test_move.cpp motor.cpp)

add_executable(grab_test grab_test.cpp motor.cpp)

target_link_libraries(test_move ${LIBSERIALPORT_LIBRARIES})

target_link_libraries(grab_test ${LIBSERIALPORT_LIBRARIES})

target_link_libraries(control ${LIBSERIALPORT_LIBRARIES} ${OpenCV_LIBS})


add_executable(camera camera.cpp)
target_link_libraries(camera ${OpenCV_LIBS})