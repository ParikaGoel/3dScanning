CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

PROJECT(exercise3)

SET(CMAKE_CXX_STANDARD 14)

find_package(Ceres REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}/libs/eigen
        ${PROJECT_SOURCE_DIR}/utils
        ${CERES_INCLUDE_DIRS})

add_executable(surface Surface/surface.cpp)
target_link_libraries(surface ceres)

add_executable(gaussian Gaussian/gaussian.cpp)
target_link_libraries(gaussian ceres)

add_executable(dragon Dragon/dragon.cpp)
target_link_libraries(dragon ceres)