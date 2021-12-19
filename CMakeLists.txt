# We use cmake 3.10 version
cmake_minimum_required(VERSION 3.10)

# Let's use C++11. For example, GoogleTest requires at least C++11
set(CMAKE_CXX_STANDARD 11)
# Different configurations for debug and release, let us look professional
set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)

# Our project name
project(kinect_fusion_project) 
find_package( OpenCV REQUIRED )
find_package(Eigen3 REQUIRED)
# Set variables here
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs CACHE PATH "Path to lib folder")

# Only for Mac OS
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(MAC_OS TRUE)

    # Add libraries for Mac OS
    include_directories(/usr/local/include)
    link_directories(/usr/local/lib)
endif()

# Add google test library for unit tests
add_subdirectory(${LIBRARY_DIR}/googletest)
include_directories(${LIBRARY_DIR}/eigen)
enable_testing() # For Google test unit tests

# Add tests
add_executable(
    DataPipelineTest # Test name
    test/DataPipelineTest.cpp # Test file
)
add_executable(
    DepthImgPreprocessingTest # Test name
    test/DepthImgPreprocessingTest.cpp # Test file
)
# Link test with google test library
target_link_libraries(
    DataPipelineTest # Out test
    gtest # Google Test library
    gtest_main
    opencv_imgproc
    opencv_highgui
    opencv_imgcodecs
)

target_link_libraries(
    DepthImgPreprocessingTest # Out test
    gtest # Google Test library
    gtest_main
    opencv_imgproc
    opencv_highgui
    opencv_imgcodecs
    Eigen3::Eigen
)

include(GoogleTest)

# Yes, we have tests!
gtest_discover_tests(DataPipelineTest)