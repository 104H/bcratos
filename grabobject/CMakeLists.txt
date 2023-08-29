cmake_minimum_required(VERSION 3.9.1)

project(bcratos_grabobject)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(move_arm src/move_arm.cpp src/common.cpp)

target_link_libraries(move_arm franka)