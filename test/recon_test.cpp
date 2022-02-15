#include <gtest/gtest.h>

#include <surface_reconstruct.h>
#include <virtual_sensor.h>
#include <voxel.h>

namespace kinect_fusion {
TEST(ReconstructTest, BasicAssertions) {
Voxel volum(100, 100, 100, 0, 0, 0, -100, 0);
VirtualSensor sensor;
std::string filenameIn{"../data/rgbd_dataset_freiburg1_xyz/"};
sensor.init(filenameIn);
const unsigned int k = 3;//TODO
unsigned int i = 0;
// const float defaut_dst = volum.getDefaultDist();//TODO: get wrong intial value
while (sensor.processNextFrame() && i <= k) {
Matrix4f depthExtri = sensor.getDepthExtrinsics();
update_volument(sensor, volum, depthExtri);
i++;
}
}
} // namespace kinect_fusion
