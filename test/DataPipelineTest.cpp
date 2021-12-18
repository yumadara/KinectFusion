#include <gtest/gtest.h>

#include <utils/include/VirtualSensor.h>
#include <utils/include/PointCloud.h>

/// @brief Basic test to check that data pipeline works
TEST(DataPipelineTest, CheckDataPipeline) {
	std::string filenameIn{"../data/rgbd_dataset_freiburg1_xyz/"};
	VirtualSensor sensor;
	
	// Initialization should be successful.
	EXPECT_TRUE(sensor.init(filenameIn));

	// Load first frame.
	sensor.processNextFrame();
	PointCloud target{sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight()};

	// Get points and check the number of point.
	std::vector<Vector3f> points{target.getPoints()};
	EXPECT_EQ(points.size(), 219100U);
}