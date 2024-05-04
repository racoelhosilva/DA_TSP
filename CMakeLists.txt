cmake_minimum_required(VERSION 3.26)
project(DA_TSP)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(DA_TSP main.cpp)

add_subdirectory(docs)
