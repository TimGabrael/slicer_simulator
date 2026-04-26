cmake_minimum_required(VERSION 3.25)
project(slicer)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

if(CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-deprecated-declarations -Wno-missing-field-initializers -Wno-unused-variable -Wno-unused-function")
    set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-deprecated-declarations -Wno-missing-field-initializers -Wno-unused-variable -Wno-unused-function")
endif()

add_subdirectory(third-party/raylib/)
include_directories(third-party/raylib/src)
add_executable(slicer src/main.cpp src/util.cpp src/util.h)

target_link_libraries(slicer raylib)
