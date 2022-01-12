#include <gtest/gtest.h>

#include <data_frame.h>

namespace kinect_fusion {

TEST(SurfaceMeasurementTest, MainTest) {
    Eigen::Matrix3f cameraIntrinstics{Eigen::Matrix3f::Zero()};

    cameraIntrinstics(0, 0) = 1.0;
    cameraIntrinstics(1, 1) = 1.0;

    cameraIntrinstics(0, 2) = 2.0;
    cameraIntrinstics(1, 2) = 2.0;

    Map2Df depth{16, 16, 1.0};

    FrameData frameData{cameraIntrinstics, 16, 16};
    frameData.updateValues(depth);

    EXPECT_NEAR(frameData.getSurface().getNormalMap().get(0, 0)(0), 0.0, 1e-6);
    EXPECT_NEAR(frameData.getSurface().getVertexMap().get(0, 0)(0), -2, 1e-6);
}

} // namespace kinect_fusion