CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

PROJECT(exercise2)

SET(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}/utils
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR})

add_executable(exercise2 main.cpp utils/Volume.cpp)