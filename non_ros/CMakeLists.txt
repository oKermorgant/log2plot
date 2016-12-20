cmake_minimum_required(VERSION 2.8.3)
project(log2plot)


add_compile_options(-std=c++11)

include_directories(../include)
add_library(log2plot   ../include/log2plot/logger.h ../src/logger.cpp)
