cmake_minimum_required(VERSION 3.9.1)

project(bcratos_grabobject)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sockpp REQUIRED)

include_directories(include)

find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

add_executable(move_arm src/main.cpp src/arm.cpp)

target_link_libraries(move_arm PRIVATE franka)
target_link_libraries(move_arm PRIVATE ${SERIAL_LDFLAGS})
target_link_libraries(move_arm PRIVATE sockpp)