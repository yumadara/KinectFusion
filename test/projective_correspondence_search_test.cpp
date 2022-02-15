#include <gtest/gtest.h>

#include <projective_correspondence_search.h>
#include <virtual_sensor.h>

namespace kinect_fusion {

// Demonstrate some basic assertions.
TEST(projectiveCorrespondenceSearchTest, BasicAssertions) {
	std::string filenameIn = std::string("/mnt/d/Users/chiyu/1 semester/KinectFusion/data/rgbd_dataset_freiburg1_xyz/");

	std::string filenameBaseOut = std::string("mesh_");

	// Load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.init(filenameIn)) {
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		//return -1;
	}

	// We store a first frame as a reference frame. All next frames are tracked relatively to the first frame.
	sensor.processNextFrame();

	Map2Df depths{sensor.getDepth()};
	std::uint32_t width = sensor.getDepthImageWidth();
	std::uint32_t height = sensor.getDepthImageHeight();
	Eigen::Matrix3f cameraIntrinsics{ sensor.getDepthIntrinsics() };
	std::cout << "frame k - 1 set up" << std::endl;
	
	int i = 0;
	const int iMax = 1;
	while (sensor.processNextFrame() && i <= 0) {
		Map2Df depths{ sensor.getDepth() };
		std::uint32_t width = sensor.getDepthImageWidth();
		std::uint32_t height = sensor.getDepthImageHeight();
		
		Eigen::Matrix3f cameraIntrinsics{ sensor.getDepthIntrinsics()};
		FrameData frame_data(cameraIntrinsics, height, width);
		frame_data.updateValues(depths);
		std::cout << "frame k set up" << std::endl;
		Matrix4f previousTransformation = Matrix4f::Identity();
		Matrix4f currentTransformation = Matrix4f::Identity();
		std::cout << "pc initial start";
		projectiveCorrespondence pc(frame_data.getSurface().getVertexMap(),
			previousTransformation,
			currentTransformation,cameraIntrinsics);
		std::cout << "pc initial end";
		pc.matchPoint();
		//std::vector<int> match = pc.getMatch();
		std::cout << "matched point size" << pc.getMatch().size()<<std::endl;
		//std::cout << "target point size" << pc.getTargetPoint().size()<<std::endl;
		i++;
	}
}
} // namespace kinect_fusion
