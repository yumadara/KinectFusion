#include <gtest/gtest.h>
#include "surface_reconstruct.h"
#include"virtual_sensor.h"
#include "Voxel.h"

namespace kinect_fusion {
TEST(ReconstructTest, BasicAssertions) {
Voxel volum(100, 100, 100, 0, 0, 0, -100, 0);
VirtualSensor sensor;
std::string filenameIn{"../data/rgbd_dataset_freiburg1_xyz/"};
sensor.init(filenameIn);
update_volument(sensor, volum);
}
} // namespace kinect_fusion
