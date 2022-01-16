#include "../src/Sensor_Pose_Estimation/include/PoseEstimator.h"
#include <gtest/gtest.h>
#include <simple_mesh.h>
namespace kinect_fusion {

	// Demonstrate some basic assertions.
	TEST(PoseEstimatorTest, BasicAssertions) {
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

		MatrixXf previousTransformation = Matrix4f::Identity();
		MatrixXf currentTransformation = Matrix4f::Identity();

		/*SimpleMesh lastDepthMesh{ sensor,  previousTransformation, 0.1f };
		std::stringstream ss;
		ss << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off";
		std::cout << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off" << std::endl;
		if (!lastDepthMesh.writeMesh(ss.str())) {
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;

		}*/

		Map2Df firstframe_depths{ sensor.getDepth() };
		std::uint32_t width = sensor.getDepthImageWidth();
		std::uint32_t height = sensor.getDepthImageHeight();
		Eigen::Matrix3f firstframe_cameraIntrinsics{ sensor.getDepthIntrinsics() };
		FrameData first_frame_data(firstframe_cameraIntrinsics, height, width);
		first_frame_data.updateValues(firstframe_depths);
		std::cout << "frame 0 set up" << std::endl;

		for (int u = 0; u < 2; u++)
		{
			sensor.processNextFrame();
		}
		Map2Df secondframe_depths{ sensor.getDepth() };
		Eigen::Matrix3f secondframe_cameraIntrinsics{ sensor.getDepthIntrinsics() };
		FrameData second_frame_data(secondframe_cameraIntrinsics, height, width);
		second_frame_data.updateValues(secondframe_depths);
		std::cout << "frame 20 set up" << std::endl;



		Map2DVector3f currentFrameNormal = second_frame_data.getSurface().getNormalMap();
		Map2DVector3f currentFrameVertex = second_frame_data.getSurface().getVertexMap();

		Map2DVector3f lastFrameNormal = first_frame_data.getSurface().getNormalMap();
		Map2DVector3f lastFrameVertex = first_frame_data.getSurface().getVertexMap();

		for (int i = 0; i != 3; i++)
		{
			std::cout << "iteration " << i << std::endl;
			projectiveCorrespondence pc(currentFrameVertex, currentFrameNormal, previousTransformation, currentTransformation, secondframe_cameraIntrinsics);
			pc.matchPoint();
			std::map<int, int> match = pc.getMatch();
			std::map<int, int>::iterator it;

			std::cout << "size of pc match" << pc.getMatch().size() << std::endl;

			PoseEstimator pose_estimator(second_frame_data, first_frame_data, previousTransformation, currentTransformation);

			Matrix4f matrix = pose_estimator.pruneCorrespondences(lastFrameNormal, currentFrameNormal, lastFrameVertex, currentFrameVertex, match);
			std::cout << "after pruning, size of pc match" << match.size() << std::endl;
			currentTransformation = matrix * currentTransformation;
			std::cout << "Current camera pose: " << currentTransformation << std::endl;
			//std::cout << "inverse of matrix" << matrix.inverse() << std::endl;
		}
		


		SimpleMesh currentDepthMesh{ sensor, currentTransformation , 0.1f };
		std::stringstream ss_current_transformed;
		ss_current_transformed << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off";
		std::cout << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off" << std::endl;
		if (!currentDepthMesh.writeMesh(ss_current_transformed.str())) {
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			
		}

		/*SimpleMesh currentDepthMeshnotTransfomed{ sensor, previousTransformation , 0.1f };
		std::stringstream ss_current_notTransfomed;
		ss_current_notTransfomed << filenameBaseOut << sensor.getCurrentFrameCnt() << "_notTransformed.off";
		std::cout << filenameBaseOut << sensor.getCurrentFrameCnt() << "_notTransformed.off" << std::endl;
		if (!currentDepthMeshnotTransfomed.writeMesh(ss_current_notTransfomed.str())) {
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;

		}*/
		

	}
} // namespace kinect_fusion
