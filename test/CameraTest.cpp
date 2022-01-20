#include <gtest/gtest.h>

#include <Camera.h>

#include "Eigen.h"

namespace kinect_fusion {

/// @brief Basic test to check that data pipeline works
TEST(CameraTest, CheckCameraConstruction) {
    
    Matrix4f pose = Matrix4f::Identity();
    Matrix3f inverseCalibrationMatrix = Matrix3f::Identity();
    int pictureHeightInPixel = 100;
    int pictureWidthInPixel = 100;
    int originXInPixel = 0;
    int originYInPixel = 0;
	Camera camera(pose, inverseCalibrationMatrix,
    pictureHeightInPixel, pictureWidthInPixel,
    originXInPixel, originYInPixel);
}


TEST(CameraTest, CheckCameraToWorld) {
	Matrix4f pose = Matrix4f::Identity();
    Matrix3f inverseCalibrationMatrix = Matrix3f::Identity();
    int pictureHeightInPixel = 100;
    int pictureWidthInPixel = 100;
    int originXInPixel = 0;
    int originYInPixel = 0;
	Camera camera(pose, inverseCalibrationMatrix,
    pictureHeightInPixel, pictureWidthInPixel,
    originXInPixel, originYInPixel);
    Vector3f cameraFrame = Vector3f(1.,0.,0.);
    Vector3f worldFrame = camera.cameraToWorld(cameraFrame);
    EXPECT_NEAR((worldFrame-cameraFrame).array().abs().sum(), 0., 0.0001);
}

} // namespace kinect_fusion

