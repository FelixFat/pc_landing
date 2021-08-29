cmake_minimum_required(VERSION 3.1)
project(proj VERSION 0.1 LANGUAGES CXX)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCE_CODE src/pc_main.cpp)
set(SOURCE_HEADER src/pc_landing.cpp src/pc_landing.h)

add_library(pc_landing STATIC ${SOURCE_HEADER})

add_executable(main ${SOURCE_CODE})
target_link_libraries(main pc_landing ${PCL_LIBRARIES})
