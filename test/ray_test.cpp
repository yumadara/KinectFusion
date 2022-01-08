#include <gtest/gtest.h>

#include <Ray.h>

namespace kinect_fusion {

/// @brief Basic test to check that Ray works
TEST(RayTest, CheckRayConstruction) {
    Vector3f direction(3.,4.,0.);
	Ray ray(20,200,400,direction);
    EXPECT_NEAR(ray.getCurrLocation()(0), 120.,0.0001);
    EXPECT_NEAR(ray.getCurrLocation()(1), 160.,0.0001);
    EXPECT_NEAR(ray.getCurrLocation()(2), 0.,0.0001);
}


TEST(VoxelTest, CheckStep) {
	Vector3f direction(3.,4.,0.);
	Ray ray(20,200,400,direction);
    ray.step();
    EXPECT_NEAR(ray.getCurrLocation()(0), 132.,0.0001);
    EXPECT_NEAR(ray.getCurrLocation()(1), 176.,0.0001);
    EXPECT_NEAR(ray.getCurrLocation()(2), 0.,0.0001);
    EXPECT_NEAR(ray.getLastLocation()(0), 120.,0.0001);
    EXPECT_NEAR(ray.getLastLocation()(1), 160.,0.0001);
    EXPECT_NEAR(ray.getLastLocation()(2), 0.,0.0001);
}

TEST(VoxelTest, CheckIsInBound) {
	Vector3f direction(3.,4.,0.);
	Ray ray(20,200,210,direction);
    
    EXPECT_TRUE(ray.isInBound());
    ray.step();
    EXPECT_FALSE(ray.isInBound());
}

} // namespace kinect_fusion

