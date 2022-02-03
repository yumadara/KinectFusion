#include <gtest/gtest.h>

#include <data_frame_cuda.h>
#include <surface_measurement_utils_cuda.h>

namespace kinect_fusion {

TEST(SurfaceMeasurementTestCuda, Assumptions) {
    // Please do not change these values
    EXPECT_EQ(FIRST_LEVEL, 0U);
    EXPECT_EQ(SECOND_LEVEL, 1U);
    EXPECT_EQ(THIRD_LEVEL, 2U);

    EXPECT_EQ(LEVELS[0], 0U);
    EXPECT_EQ(LEVELS[1], 1U);
    EXPECT_EQ(LEVELS[2], 2U);

    EXPECT_EQ(NUMBER_OF_LEVELS, 3U);
}

TEST(SurfaceMeasurementTestCuda, MainTest) {
    Eigen::Matrix3f cameraIntrinstics{Eigen::Matrix3f::Zero()};

    cameraIntrinstics(0, 0) = 1.0;
    cameraIntrinstics(1, 1) = 1.0;

    cameraIntrinstics(0, 2) = 2.0;
    cameraIntrinstics(1, 2) = 2.0;

    Map2Df depth{16, 16, 1.0};

    FrameDataCuda frameData{cameraIntrinstics, 16, 16};
    frameData.updateValues(depth);

    EXPECT_NEAR(frameData.getSurface().getNormalMap().get(0, 0)(0), 0.0, 1e-6);
    EXPECT_NEAR(frameData.getSurface().getVertexMap().get(0, 0)(0), -2, 1e-6);
}

TEST(SurfaceMeasurementTestCuda, SubsampleTest) {
    Map2DfCuda depth{4, 4};
    for (std::size_t i = 0; i < 16; i++) {
        depth[i] = float(i) * 1e-3;
    }

    Map2DfCuda subsampledDepth{2, 2};
    subsample(depth, subsampledDepth);

    EXPECT_NEAR(subsampledDepth.get(0, 0), (0.0 + 1.0 + 4.0 + 5.0) / 4.0 * 1e-3, 1e-6);
    EXPECT_NEAR(subsampledDepth.get(0, 1), (1.0 + 2.0 + 3.0 + 5.0 + 6.0 + 7.0) / 6.0 * 1e-3, 1e-6);
    EXPECT_NEAR(subsampledDepth.get(1, 0), (4.0 + 5.0 + 8.0 + 9.0 + 12.0 + 13.0) / 6.0 * 1e-3, 1e-6);
    EXPECT_NEAR(subsampledDepth.get(1, 1), (5.0 + 6.0 + 7.0 + 9.0 + 10.0 + 11.0 + 13.0 + 14.0 + 15.0) / 9.0 * 1e-3, 1e-6);
}

TEST(SurfaceMeasurementTestCuda, SubsampleTestBigValues) {
    Map2DfCuda depth{4, 4};
    for (std::size_t i = 0; i < 16; i++) {
        depth[i] = float(i) * 1e-3;
    }
    depth[5] = MINF;
    depth[15] = 100; 

    Map2DfCuda subsampledDepth{2, 2};
    subsample(depth, subsampledDepth);

    EXPECT_NEAR(subsampledDepth.get(0, 0), (0.0 + 1.0 + 4.0) / 3.0 * 1e-3, 1e-6);
    EXPECT_NEAR(subsampledDepth.get(0, 1), (1.0 + 2.0 + 3.0 + 6.0 + 7.0) / 5.0 * 1e-3, 1e-6);
    EXPECT_NEAR(subsampledDepth.get(1, 0), (4.0 + 8.0 + 9.0 + 12.0 + 13.0) / 5.0 * 1e-3, 1e-6);
    EXPECT_NEAR(subsampledDepth.get(1, 1), (6.0 + 7.0 + 9.0 + 10.0 + 11.0 + 13.0 + 14.0) / 7.0 * 1e-3, 1e-6);
}

TEST(SurfaceMeasurementTestCuda, SubsampleInvalidDepths) {
    Map2DfCuda depth{4, 4};
    for (std::size_t i = 0; i < 16; i++) {
        depth[i] = float(i) * 1e-3;
    }
    depth[0] = MINF;
    depth[2] = MINF;

    Map2DfCuda subsampledDepth{2, 2};
    subsample(depth, subsampledDepth);

    EXPECT_EQ(subsampledDepth.get(0), MINF);
    EXPECT_EQ(subsampledDepth.get(1), MINF);
}
} // namespace kinect_fusion