# We use cmake 3.10 version
cmake_minimum_required(VERSION 3.10)
set(CMAKE_BUILD_TYPE Debug)
# Let's use C++11. For example, GoogleTest requires at least C++11
set(CMAKE_CXX_STANDARD 11)
# Different configurations for debug and release, let us look professional
set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)
IF(CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)

# Create compile_commands.json in build for visual studio code
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Our project name
project(kinect_fusion_project) 

# FInd important libraries
#find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Set variables here
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs CACHE PATH "Path to lib folder")
set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src CACHE PATH "Path to src folder")
#set(FreeImage_LIBRARY_DIR ${LIBRARY_DIR}/FreeImage-3.18.0/Dist/x64/ CACHE PATH "Path to free image library")

if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(FreeImage_INCLUDE_DIR ${LIBRARY_DIR}/FreeImage-3.18.0/Dist/x64/ CACHE PATH "Path to FreeImage header file")
        set(FreeImage_LIBRARY_DIR ${LIBRARY_DIR}/FreeImage-3.18.0/Dist/x64/ CACHE PATH "Path to FreeImage .lib/.dll folder")
    elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(FreeImage_INCLUDE_DIR ${LIBRARY_DIR}/FreeImage-3.18.0/Dist/x32/ CACHE PATH "Path to FreeImage header file")
        set(FreeImage_LIBRARY_DIR ${LIBRARY_DIR}/FreeImage-3.18.0/Dist/x32/ CACHE PATH "Path to FreeImage .lib/.dll folder")
    endif()
endif(WIN32)

set(Eigen3_DIR ${LIBRARY_DIR}/Eigen/share/eigen3/cmake CACHE PATH "Path to installed Eigen")

# Only for Mac OS
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(MAC_OS TRUE)

    # Add libraries for Mac OS
    include_directories(/usr/local/include)
    link_directories(/usr/local/lib)
endif()

# Add source directories for includes
include_directories(${SOURCE_DIR})
include_directories(${SOURCE_DIR}/utils/include) # We need it for legacy code
include_directories(${SOURCE_DIR}/data_preprocessing/include)
include_directories(${SOURCE_DIR}/ray_casting/include)

# Define our source files
set(SOURCES ${SOURCE_DIR}/utils/src/free_image_helper.cpp
            ${SOURCE_DIR}/data_preprocessing/src/data_frame.cpp)

# Add libraries directories for includes
include_directories(${LIBRARY_DIR}/eigen)
include_directories(${OpenCV_INCLUDE_DIRS})

# Link libraries here
link_directories(${FreeImage_LIBRARY_DIR})

# Add google test library for unit tests
add_subdirectory(${LIBRARY_DIR}/googletest)

enable_testing() # For Google test unit tests

# Add tests

add_executable(
    DataPipelineTest # Test name
    test/data_pipeline_test.cpp # Test file
    ${SOURCES}
)

add_executable(
    DepthImgPreprocessingTest # Test name
    test/DepthImgPreprocessingTest.cpp # Test file
    ${SOURCES}
)
# add_executable(
#     VoxelTest # Test name
#     test/VoxelTest.cpp # Test file
#     ${SOURCES}
# )
add_executable(
    RayTest 
    test/ray_test.cpp
    ${SOURCES}
)
add_executable(
    CameraTest
    test/CameraTest.cpp
    ${SOURCES}
)
add_executable(
    SurfaceTest
    test/SurfaceTest.cpp
    ${SOURCES}
)

# add_executable(
#     RayCastingTest
#     test/RayCastingTest.cpp
#     ${SOURCES}
# )
add_executable(
    ReconstructTest
    test/recon_test.cpp
    ${SOURCES}
)
#enable_testing()


# Link test with google test library
target_link_libraries(
    DataPipelineTest # Our test
    gtest # Google Test library
    gtest_main
    Eigen3::Eigen
    freeimage
    ${OpenCV_LIBS}
)

target_link_libraries(
    DepthImgPreprocessingTest # Out test
    gtest # Google Test library
    gtest_main
    freeimage
    Eigen3::Eigen
    ${OpenCV_LIBS}
)
target_link_libraries(
    RayTest # Out test
    gtest # Google Test library
    gtest_main
    freeimage
    Eigen3::Eigen
    ${OpenCV_LIBS}
)
target_link_libraries(
    CameraTest # Out test
    gtest # Google Test library
    gtest_main
    freeimage
    Eigen3::Eigen
    ${OpenCV_LIBS}
)
target_link_libraries(
    SurfaceTest # Out test
    gtest # Google Test library
    gtest_main
    freeimage
    Eigen3::Eigen
    ${OpenCV_LIBS}
)
# target_link_libraries(
#     RayCastingTest # Out test
#     gtest # Google Test library
#     gtest_main
#     freeimage
#     Eigen3::Eigen
#     ${OpenCV_LIBS}
# )
# target_link_libraries(
#     VoxelTest # Out test
#     gtest # Google Test library
#     gtest_main
#     freeimage
#     Eigen3::Eigen
#     ${OpenCV_LIBS}
# )

target_link_libraries(
    ReconstructTest # Out test
    gtest # Google Test library
    gtest_main
    freeimage
    Eigen3::Eigen
    ${OpenCV_LIBS}
)

# Yes, we have tests!
include(GoogleTest)
gtest_discover_tests(DataPipelineTest)

