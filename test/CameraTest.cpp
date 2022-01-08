#include <gtest/gtest.h>

#include <camera.h>

#include "Eigen.h"

/// @brief Basic test to check that data pipeline works
TEST(CameraTest, CheckCameraConstruction) {
    Vector3f translation = Vector3f(0.,0.,0.);
    Matrix3f rotation = Matrix3f::Identity();
    Matrix3f inverseCalibrationMatrix = Matrix3f::Identity();
    int pictureHeightInPixel = 100;
    int pictureWidthInPixel = 100;
    int originXInPixel = 0;
    int originYInPixel = 0;
	Camera camera(translation,rotation, inverseCalibrationMatrix,
    pictureHeightInPixel, pictureWidthInPixel,
    originXInPixel, originYInPixel);
}


TEST(CameraTest, CheckCameraToWorld) {
	Vector3f translation = Vector3f(0.,0.,0.);
    Matrix3f rotation = Matrix3f::Identity();
    Matrix3f inverseCalibrationMatrix = Matrix3f::Identity();
    int pictureHeightInPixel = 100;
    int pictureWidthInPixel = 100;
    int originXInPixel = 0;
    int originYInPixel = 0;
	Camera camera(translation,rotation, inverseCalibrationMatrix,
    pictureHeightInPixel, pictureWidthInPixel,
    originXInPixel, originYInPixel);
    Vector3f cameraFrame = Vector3f(1.,0.,0.);
    Vector3f worldFrame = camera.cameraToWorld(cameraFrame);
    EXPECT_NEAR((worldFrame-cameraFrame).array().abs().sum(), 0., 0.0001);
}

