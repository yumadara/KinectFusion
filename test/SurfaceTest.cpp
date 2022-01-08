#include <gtest/gtest.h>

#include <surface.h>


TEST(SurfaceTest, CheckSurfaceConstruction) {
    Camera camera;
	Surface surface(camera);
}


TEST(SurfaceTest, CheckSetterGetter) {
	Camera camera;
	Surface surface(camera);
    Vector3f testValue  = Vector3f(1,0,0);
    surface.setVertex(1,1,testValue);
    EXPECT_NEAR((surface.getVertex(1,1)- testValue).array().abs().sum(), 0., 0.0001);
    surface.setNormal(1,1,testValue);
    EXPECT_NEAR((surface.getNormal(1,1)- testValue).array().abs().sum(), 0., 0.0001);
}



