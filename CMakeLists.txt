cmake_minimum_required(VERSION 3.12)
project(DSPProject CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable CTest at the top level so tests are discoverable from build/
enable_testing()

# Build the sensor interface library (independent)
add_subdirectory(lib)

# Build the executable (depends on library)
add_subdirectory(src)

# Tests
add_subdirectory(tests)

