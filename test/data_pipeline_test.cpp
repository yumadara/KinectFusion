#include <gtest/gtest.h>

#include <virtual_sensor.h>

namespace kinect_fusion {

/// @brief Basic test to check that data pipeline works
TEST(DataPipelineTest, CheckDataPipeline) {
	std::string filenameIn{"../data/rgbd_dataset_freiburg1_xyz/"};
	VirtualSensor sensor;
	
	// Initialization should be successful.
	EXPECT_TRUE(sensor.init(filenameIn));

	// Load first frame.
	sensor.processNextFrame();

	Map2Df depths{sensor.getDepth()};
	std::uint32_t width = sensor.getDepthImageWidth();
	std::uint32_t height = sensor.getDepthImageHeight();

	Eigen::Matrix3f cameraIntrinsics{sensor.getDepthIntrinsics()};

	// Check dimensions
	EXPECT_EQ(width, 640);
	EXPECT_EQ(height, 480);

	//	Check row major property
	for (std::size_t row = 0; row < height; row++)
	{
		for (std::size_t col = 0; col < width; col++)
		{
			// Definition of row majority.
			EXPECT_EQ(depths.get(row, col), depths.data()[row * width + col]);
		}
	}

	// Check first element
	EXPECT_EQ(depths.data()[0], MINF);

	// Check number of depths.
	EXPECT_EQ(depths.size(), /* 640 * 480 = */ 307200);

	// Check number of MINF, so not valid depths.
	std::vector<float> depthsAsVector{depths.getDataVector()};
	EXPECT_EQ(std::count(depthsAsVector.begin(), depthsAsVector.end(), MINF), 77325);

	// Check number of unique depths.
	std::set<float> unique_depths(depthsAsVector.begin(), depthsAsVector.end());
	EXPECT_EQ(unique_depths.size(), 289);

	EXPECT_NE(cameraIntrinsics, Eigen::Matrix3f::Zero());
}
} // namespace kinect_fusion