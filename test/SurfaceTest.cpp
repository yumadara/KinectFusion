#include <gtest/gtest.h>

#include <Surface.h>

namespace kinect_fusion {

TEST(SurfaceTest, CheckSurfaceConstruction) {
	Surface surface(2U, 2U);
}


TEST(SurfaceTest, CheckSetterGetter) {
	Surface surface(2U, 2U);
    Vector3f testValue  = Vector3f(1,0,0);
    surface.setVertex(1,1,testValue);
    EXPECT_NEAR((surface.getVertex(1,1)- testValue).array().abs().sum(), 0., 0.0001);
    surface.setNormal(1,1,testValue);
    EXPECT_NEAR((surface.getNormal(1,1)- testValue).array().abs().sum(), 0., 0.0001);
}

} // namespace kinect_fusion



