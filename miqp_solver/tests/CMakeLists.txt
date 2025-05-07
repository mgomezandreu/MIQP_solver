# Create a test target and link GTest + your code
add_executable(miqp_tests test_miqp.cpp)
target_link_libraries(miqp_tests gtest_main)

# Link your project’s source code to the test target
target_include_directories(miqp_tests PRIVATE ../include)
#find all files in the src directory

# Register the test with CMake
include(GoogleTest)
gtest_discover_tests(miqp_tests)