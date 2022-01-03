<<<<<<< HEAD
#include <gtest/gtest.h>
#include "surface_reconstruct.h"
#include"virtual_sensor.h"
#include "Voxel.h"

TEST(ReconstructTest, BasicAssertions) {
Voxel volum(100, 100, 100, 0, 0, 0, -1000, 0);
kinect_fusion::VirtualSensor sensor;
std::string filenameIn{"../data/rgbd_dataset_freiburg1_xyz/"};
sensor.init(filenameIn);
kinect_fusion::update_volument(sensor, volum);


// kinect_fusion::Voxel volum(100, 100, 100, 0, 0, 0);
=======
// #include <gtest/gtest.h>
// #include "surface_reconstruct.h"
// #include"virtual_sensor.h"kinect_fusion
// #include "Voxel.h"

// TEST(ReconstructTest, BasicAssertions) {
// kinect_fusion::Voxel volum(100, 100, 100, 0, 0, 0, -1000, 0);
>>>>>>> e8df95e20759c3de156e182b00c48a972bc37f56
// kinect_fusion::VirtualSensor sensor;
// std::string filenameIn{"../data/rgbd_dataset_freiburg1_xyz/"};
// sensor.init(filenameIn);
// kinect_fusion::update_volument(sensor, volum);


// // kinect_fusion::Voxel volum(100, 100, 100, 0, 0, 0);
// // kinect_fusion::VirtualSensor sensor;
// // std::string filenameIn{"../data/rgbd_dataset_freiburg1_xyz/"};
// // sensor.init(filenameIn);
// // kinect_fusion::update_volument(sensor, volum);

// }
