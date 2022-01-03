#include <gtest/gtest.h>
#include "../src/Sensor_Pose_Estimation/include/ProjectiveCorrespondenceSearch.h"
#include "../src/utils/include/point_cloud.h"

// Demonstrate some basic assertions.
TEST(DepthImgPreprocessingTest, BasicAssertions) {
<<<<<<< HEAD
	std::string filenameIn = std::string("../data/rgbd_dataset_freiburg1_xyz/");
=======
	std::string filenameIn = std::string("/mnt/d/Users/chiyu/1 semester/KinectFusion/data/rgbd_dataset_freiburg1_xyz/");
>>>>>>> 8b3b20528207f1d76f0a1bafcc90c6aa59bbf235
	std::string filenameBaseOut = std::string("mesh_");

	// Load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	kinect_fusion::VirtualSensor sensor;
	if (!sensor.init(filenameIn)) {
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		//return -1;
	}

	// We store a first frame as a reference frame. All next frames are tracked relatively to the first frame.
	sensor.processNextFrame();
	kinect_fusion::PointCloud target(sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight());

	int i = 0;
	const int iMax = 1;
	while (sensor.processNextFrame() && i <= 0) {
		auto depthMap = sensor.getDepth();
		Matrix3f depthIntrinsics = sensor.getDepthIntrinsics();
		Matrix4f depthExtrinsics = sensor.getDepthExtrinsics();
		kinect_fusion::PointCloud source(sensor.getDepth(), sensor.getDepthIntrinsics(),
			sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(),
			sensor.getDepthImageHeight());

		Matrix4f previousTransformation = Matrix4f::Identity();
		Matrix4f currentTransformation = Matrix4f::Identity();
		
		projectiveCorrespondence pc(target.getPoints(),
			previousTransformation,
			currentTransformation,sensor);
		pc.matchPoint();
		//std::vector<int> match = pc.getMatch();
		std::cout << "matched point size" << pc.getMatch().size()<<std::endl;
		std::cout << "target point size" << pc.getTargetPoint().size()<<std::endl;
		i++;
	}
}
