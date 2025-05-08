include(FetchContent)

cmake_minimum_required(VERSION 3.10)
project(MIQPSolver)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add include directory
include_directories(include)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

# Collect all source files
file(GLOB SOURCES "src/*.cpp")

# Build the executable
add_executable(miqp_solver main.cpp ${SOURCES})
target_link_libraries(miqp_solver PRIVATE OsqpEigen::OsqpEigen Eigen3::Eigen)



# Download GTest automatically
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/heads/main.zip
)

# Make sure it’s only downloaded and built once
FetchContent_MakeAvailable(googletest)

# Add the test directory
enable_testing()
add_subdirectory(tests)
