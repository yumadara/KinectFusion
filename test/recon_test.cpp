// <<<<<<< HEAD
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
}


// // kinect_fusion::Voxel volum(100, 100, 100, 0, 0, 0);
// =======
// // #include <gtest/gtest.h>
// // #include "surface_reconstruct.h"
// // #include"virtual_sensor.h"kinect_fusion
// // #include "Voxel.h"

// // TEST(ReconstructTest, BasicAssertions) {
// // kinect_fusion::Voxel volum(100, 100, 100, 0, 0, 0, -1000, 0);
// <<<<<<< HEAD
// >>>>>>> 8d95ff6... Almost finish data frame implementation (surface measurement)
// =======
// >>>>>>> e8df95e20759c3de156e182b00c48a972bc37f56
// >>>>>>> 8b3b20528207f1d76f0a1bafcc90c6aa59bbf235
// // kinect_fusion::VirtualSensor sensor;
// // std::string filenameIn{"../data/rgbd_dataset_freiburg1_xyz/"};
// // sensor.init(filenameIn);
// // kinect_fusion::update_volument(sensor, volum);


// // kinect_fusion::Voxel volum(100, 100, 100, 0, 0, 0);
// // kinect_fusion::VirtualSensor sensor;
// // std::string filenameIn{"../data/rgbd_dataset_freiburg1_xyz/"};
// // sensor.init(filenameIn);
// // kinect_fusion::update_volument(sensor, volum);

// }
