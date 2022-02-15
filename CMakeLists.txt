# set(CMAKE_OSX_SYSROOT "/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk")
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

# Find important libraries
find_package(OpenCV REQUIRED)
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
include_directories(${SOURCE_DIR}/surface_measurement/include)
include_directories(${SOURCE_DIR}/ray_casting/include)
include_directories(${SOURCE_DIR}/surface_reconstruction/include)
include_directories(${SOURCE_DIR}/sensor_pose_estimation/include)

# Define our source files
set(SOURCES ${SOURCE_DIR}/utils/src/free_image_helper.cpp
            ${SOURCE_DIR}/surface_measurement/src/data_frame.cpp
            ${SOURCE_DIR}/surface_measurement/src/surface_measurement_utils.cpp)

# Add libraries directories for includes
include_directories(${LIBRARY_DIR}/eigen)
include_directories(${OpenCV_INCLUDE_DIRS})

# Link libraries here
link_directories(${FreeImage_LIBRARY_DIR})

# Add google test library for unit tests
add_subdirectory(${LIBRARY_DIR}/googletest)

enable_testing() # For Google test unit tests

find_package(CUDA 8.0)
IF (CUDA_FOUND)
    include_directories("${CUDA_INCLUDE_DIRS}")
    # Optional: Specify the arch of your CUDA hardware here
    # SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3;-std=c++11 -gencode arch=compute_20,code=sm_21)
    SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-std=c++11)
    MESSAGE("CUDA was found. Hurra!!!!")
    set(CUDA_SOURCE_DIR ${SOURCE_DIR}/cuda)

    include_directories(${CUDA_SOURCE_DIR})
    include_directories(${CUDA_SOURCE_DIR}/utils/include) # We need it for legacy code
    include_directories(${CUDA_SOURCE_DIR}/surface_measurement/include)

    set(SOURCES ${SOURCES} 
                ${CUDA_SOURCE_DIR}/surface_measurement/src/data_frame_cuda.cu
                ${CUDA_SOURCE_DIR}/surface_measurement/src/surface_measurement_cuda.cu)
ENDIF ()

# Add executables
add_executable(
   SurfaceMeasurementApp # Name
   ${CMAKE_CURRENT_SOURCE_DIR}/executables/surface_measurement_app.cpp # File
   ${SOURCES}
)

target_link_libraries(
   SurfaceMeasurementApp
   freeimage
   Eigen3::Eigen
  ${OpenCV_LIBS}
)

add_executable(
   PoseEstimationApp # Name
   ${CMAKE_CURRENT_SOURCE_DIR}/executables/pose_estimation_app.cpp # File
   ${SOURCES}
)

target_link_libraries(
   PoseEstimationApp
   freeimage
   Eigen3::Eigen
  ${OpenCV_LIBS}
)

add_executable(
    KinectFusionApp # Name
    ${CMAKE_CURRENT_SOURCE_DIR}/executables/main.cpp # File
    ${SOURCES}
)

target_link_libraries(
    KinectFusionApp
    freeimage
    Eigen3::Eigen
   ${OpenCV_LIBS}
)

# Add tests
add_executable(
    DataPipelineTest # Test name
    test/data_pipeline_test.cpp # Test file
    ${SOURCES}
)

add_executable(
    PoseEstimatorTest # Test name
    test/pose_estimator_test.cpp # Test file
    ${SOURCES}
)

add_executable(
    RayTest 
    test/ray_test.cpp
    ${SOURCES}
)
add_executable(
    CameraTest
    test/camera_test.cpp
    ${SOURCES}
)
add_executable(
    SurfaceTest
    test/surface_test.cpp
    ${SOURCES}
)

add_executable(
    RayCastingTest
    test/ray_casting_test.cpp
    ${SOURCES}
)

add_executable(
    VoxelTest # Test name
    test/voxel_test.cpp # Test file
    ${SOURCES}
)

add_executable(
   ReconstructTest
   test/recon_test.cpp
   ${SOURCES}
)
add_executable(
   SurfaceMeasurementTest
   test/surface_measurement_test.cpp
   ${SOURCES}
)


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
    PoseEstimatorTest # Our test
    gtest # Google Test library
    gtest_main
    Eigen3::Eigen
    freeimage
    ${OpenCV_LIBS}
)

target_link_libraries(
    ReconstructTest # Our test
    gtest # Google Test library
    gtest_main
    Eigen3::Eigen
    freeimage
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

target_link_libraries(
    RayCastingTest # Out test
    gtest # Google Test library
    gtest_main
    freeimage
    Eigen3::Eigen
    ${OpenCV_LIBS}
)

target_link_libraries(
    VoxelTest # Out test
    gtest # Google Test library
    gtest_main
    freeimage
    Eigen3::Eigen
    ${OpenCV_LIBS}
)

target_link_libraries(
    SurfaceMeasurementTest
    gtest # Google Test library
    gtest_main
    freeimage
    Eigen3::Eigen
   ${OpenCV_LIBS}
)

# Yes, we have tests!
include(GoogleTest)
enable_testing()
gtest_discover_tests(DataPipelineTest)
gtest_discover_tests(SurfaceMeasurementTest)
# gtest_discover_tests(ReconstructTest)


