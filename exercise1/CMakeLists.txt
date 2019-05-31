CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

PROJECT(exercise1)

SET(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_library(freeimageplus REQUIRED)
find_library(freeimage REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}/utils
        ${EIGEN3_INCLUDE_DIR})

add_executable(exercise1 main.cpp ${PROJECT_SOURCE_DIR}/utils/FreeImageHelper.cpp)
target_link_libraries(exercise1 freeimageplus freeimage)