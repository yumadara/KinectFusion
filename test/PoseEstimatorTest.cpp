#include "../src/Sensor_Pose_Estimation/include/PoseEstimator.h"
#include <gtest/gtest.h>
#include <simple_mesh.h>
namespace kinect_fusion {
	TEST(PoseEstimatorTest, checkPoseEstimation) {
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
		for (int i = 0; i < 1; i++)
		{
			sensor.processNextFrame();
		}

		MatrixXf previousTransformation = Matrix4f::Identity();
		MatrixXf currentTransformation = Matrix4f::Identity();

		std::stringstream ss_first;
		ss_first << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off";
		std::cout << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off" << std::endl;
		SimpleMesh lastDepthMesh{ sensor, previousTransformation, 0.1f };
		lastDepthMesh.writeMesh(ss_first.str());

		Map2Df firstframe_depths{ sensor.getDepth() };
		std::uint32_t width = sensor.getDepthImageWidth();
		std::uint32_t height = sensor.getDepthImageHeight();
		Eigen::Matrix3f firstframe_cameraIntrinsics{ sensor.getDepthIntrinsics() };
		FrameData first_frame_data(firstframe_cameraIntrinsics, height, width);
		first_frame_data.updateValues(firstframe_depths);
		//first_frame_data.printDataFrame();
		sensor.processNextFrame();

		Map2Df secondframe_depths{ sensor.getDepth() };
		Eigen::Matrix3f secondframe_cameraIntrinsics{ sensor.getDepthIntrinsics() };
		FrameData second_frame_data(secondframe_cameraIntrinsics, height, width);
		second_frame_data.updateValues(secondframe_depths);


		PoseEstimator pose_estimator(second_frame_data, first_frame_data, previousTransformation, currentTransformation);
		//std::cout << "current transformation" <<pose_estimator.getCurrentTransformation()<<std::endl;
		std::cout << "second frame data second level vertex map size" << second_frame_data.getSurface(kinect_fusion::Level::Third).getVertexMap().size() << std::endl;
		currentTransformation = pose_estimator.frame2frameEstimation(previousTransformation);
		std::cout << "Final Transformation" << currentTransformation << std::endl;

		std::stringstream ss;
		ss << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off";
		std::cout << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off" << std::endl;
		SimpleMesh currentDepthMesh{ sensor, currentTransformation, 0.1f };
		currentDepthMesh.writeMesh(ss.str());

		std::stringstream ss_current_frame_not_transformed;
		ss_current_frame_not_transformed << filenameBaseOut << sensor.getCurrentFrameCnt() << "_not_transformed.off";
		std::cout << filenameBaseOut << sensor.getCurrentFrameCnt() << "_not_transformed.off" << std::endl;
		SimpleMesh currentDepthMeshNotTransformed{ sensor,previousTransformation , 0.1f };
		currentDepthMeshNotTransformed.writeMesh(ss_current_frame_not_transformed.str());

}
} // namespace kinect_fusion


