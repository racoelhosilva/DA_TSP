cmake_minimum_required(VERSION 3.26)
project(DA_TSP)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -pedantic")
set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -Werror -Wall -pedantic")

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(DA_TSP main.cpp
        include/Interface.h
        src/Interface.cpp
        include/input.h
        src/input.cpp
        include/ansi.h
        include/Graph.h
        src/Graph.cpp
        include/Edge.h
        src/Edge.cpp
        include/Vertex.h
        src/Vertex.cpp
        src/Ufds.cpp
        include/Ufds.h
)

add_subdirectory(docs)
