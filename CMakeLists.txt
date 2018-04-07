cmake_minimum_required(VERSION 2.4.0)

project(DttSP)

if ( CMAKE_COMPILER_IS_GNUCC )
    set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -std=gnu99")
endif()

# Find source files
file(GLOB SOURCES *.c)

# Include header files
include_directories(.)


# Create shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Link with libs
target_link_libraries(${PROJECT_NAME} lo pthread m jack fftw3f c db dl)

# Install library
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

