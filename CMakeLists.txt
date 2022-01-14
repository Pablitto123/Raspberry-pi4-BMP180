cmake_minimum_required(VERSION 3.17)
project(czujnik_temp)

set(CMAKE_CXX_STANDARD 14)

add_executable(czujnik_temp main.cpp src/BMP180.cpp)