cmake_minimum_required(VERSION 3.9.1)

set (CMAKE_CXX_STANDARD 20)

project(bcratos_grabobject)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sockpp REQUIRED)

configure_file(src/config.yaml config.yaml COPYONLY)

include_directories(include)

find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG yaml-cpp-0.7.0 
)
FetchContent_GetProperties(yaml-cpp)

if(NOT yaml-cpp_POPULATED)
  message(STATUS "Fetching yaml-cpp...")
  FetchContent_Populate(yaml-cpp)
  add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
endif()

add_executable(move_arm src/main.cpp src/arm.cpp)

target_link_libraries(move_arm PRIVATE franka)
target_link_libraries(move_arm PRIVATE ${SERIAL_LDFLAGS})
target_link_libraries(move_arm PRIVATE sockpp)
target_link_libraries(move_arm PRIVATE yaml-cpp::yaml-cpp)