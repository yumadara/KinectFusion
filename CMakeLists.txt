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
set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src CACHE PATH "Path to src folder")
set(FreeImage_LIBRARY_DIR ${LIBRARY_DIR}/FreeImage-3.18.0/Dist/x64/ CACHE PATH "Path to free image library")

# Only for Mac OS
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(MAC_OS TRUE)

    # Add libraries for Mac OS
    include_directories(/usr/local/include)
    link_directories(/usr/local/lib)
endif()

find_package(Eigen3 REQUIRED)

# Add source directories for includes
include_directories(${SOURCE_DIR})
include_directories(${SOURCE_DIR}/utils/include) # We need it for legacy code
include_directories(${SOURCE_DIR}/data_preprocessing/include)

# Define our source files
set(SOURCES ${SOURCE_DIR}/utils/src/free_image_helper.cpp
            ${SOURCE_DIR}/data_preprocessing/src/data_frame.cpp)

# Add libraries directories for includes
include_directories(${LIBRARY_DIR}/eigen)

# Link libraries here
link_directories(${FreeImage_LIBRARY_DIR})

# Add google test library for unit tests
add_subdirectory(${LIBRARY_DIR}/googletest)
include_directories(${LIBRARY_DIR}/eigen)
enable_testing() # For Google test unit tests

# Add tests
add_executable(
    DataPipelineTest # Test name
    test/VoxelTest.cpp
    test/data_pipeline_test.cpp # Test file
    ${SOURCES}
)

add_executable(
    DepthImgPreprocessingTest # Test name
    test/DepthImgPreprocessingTest.cpp # Test file
)

#enable_testing()
#add_executable(
#    VoxelTest # Test name
#    test/VoxelTest.cpp # Test file
#    ${SOURCES}
#)


# Link test with google test library
target_link_libraries(
    DataPipelineTest # Our test
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
    freeimage
    Eigen3::Eigen
)

# Yes, we have tests!
include(GoogleTest)
gtest_discover_tests(DataPipelineTest)
