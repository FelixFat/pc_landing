cmake_minimum_required(VERSION 3.1)
project(pc_landing VERSION 0.1 LANGUAGES CXX)

set(SOURCE_CODE src/pc_main.cpp)
set(SOURCE_HEADER src/pc_landing.cpp)

add_library(pc_landing STATIC ${SOURCE_HEADER})

add_executable(pc_landing ${SOURCE_CODE})
target_link_libraries(pc_landing head)
