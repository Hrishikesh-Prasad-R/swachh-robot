cmake_minimum_required(VERSION 3.10)
project(SwacchHMI VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find GTK4
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK4 REQUIRED gtk4)

# Add executable
add_executable(swacch_hmi hmi_main.cpp)

# Include directories
target_include_directories(swacch_hmi PRIVATE ${GTK4_INCLUDE_DIRS})

# Link directories
target_link_directories(swacch_hmi PRIVATE ${GTK4_LIBRARY_DIRS})

# Link libraries
target_link_libraries(swacch_hmi PRIVATE ${GTK4_LIBRARIES})

# Compiler flags
target_compile_options(swacch_hmi PRIVATE 
    ${GTK4_CFLAGS_OTHER}
    -Wall
    -O3
    -march=native
)

# Install target
install(TARGETS swacch_hmi DESTINATION bin)
