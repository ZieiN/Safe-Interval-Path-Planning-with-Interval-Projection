cmake_minimum_required(VERSION 3.20)
project(A_TI)

set( CMAKE_CXX_STANDARD 23 )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -Wextra" )


add_executable(A_TI
       # A-TI.cpp
       # A-star.cpp
        SIPP.cpp
#        generate_obstacles.cpp
        )
