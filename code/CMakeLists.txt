cmake_minimum_required(VERSION 3.10)
project(SwacchHMI VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find GTK4
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK4 REQUIRED gtk4)

# Source files
set(SOURCES
    main.cpp
    serial_comm.cpp
    callbacks.cpp
    gui.cpp
)

# Add executable
add_executable(hmi ${SOURCES})

# Include directories
target_include_directories(hmi PRIVATE 
    ${GTK4_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link directories
target_link_directories(hmi PRIVATE ${GTK4_LIBRARY_DIRS})

# Link libraries
target_link_libraries(hmi PRIVATE ${GTK4_LIBRARIES})

# Compiler flags
target_compile_options(hmi PRIVATE 
    ${GTK4_CFLAGS_OTHER}
    -Wall
    -O3
    -march=native
)

# Install target
install(TARGETS hmi DESTINATION bin)
