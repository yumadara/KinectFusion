#include "../src/Sensor_Pose_Estimation/include/PoseEstimator.h"
#include <gtest/gtest.h>
#include <simple_mesh.h>
namespace kinect_fusion {
	TEST(PoseEstimatorTest, checkPoseEstimation) {
        std::string filenameIn{ "/mnt/d/Users/chiyu/1 semester/KinectFusion/data/rgbd_dataset_freiburg1_xyz/" };
        
        VirtualSensor sensor;
        sensor.init(filenameIn);
        //Voxel volum(200, 200, 300, 100, 100, 0, -100, 0);
        FrameData previous_dataFrame{ sensor.getDepthIntrinsics(), sensor.getDepthImageHeight(), sensor.getDepthImageWidth() };

        sensor.processNextFrame();
        Map2Df depth = sensor.getDepth();
        previous_dataFrame.updateValues(depth);

        MatrixXf previousTransformation = Matrix4f::Identity();
        MatrixXf currentTransformation = Matrix4f::Identity();

        std::string frameZero{ std::string("./mesh_without_raycasting/""frame_0_not_transformed") + std::string(".off") };
        SimpleMesh meshZero{ previous_dataFrame.getSurface().getVertexMap(), previousTransformation, 0.1f };
        meshZero.writeMesh(frameZero);

        for (unsigned int i = 1; i <4; i++) {
            //currentTransformation = 
            std::cout << "Now we are at frame " << i << std::endl;
            sensor.processNextFrame();
            FrameData current_dataFrame{ sensor.getDepthIntrinsics(), sensor.getDepthImageHeight(), sensor.getDepthImageWidth() };
            Map2Df depth = sensor.getDepth();
            current_dataFrame.updateValues(depth);

            PoseEstimator pose_estimator(current_dataFrame, previous_dataFrame, previousTransformation, currentTransformation, sensor);

            currentTransformation = pose_estimator.frame2frameEstimation(previousTransformation);


                std::string frameK{ std::string("./mesh_without_raycasting/frame_" + std::to_string(i) + "_transformed_test") + std::string(".off") };
                SimpleMesh meshK{ previous_dataFrame.getSurface().getVertexMap(), currentTransformation, 0.1f };
                meshK.writeMesh(frameK);
                meshZero = SimpleMesh::joinMeshes(meshZero, meshK);
            


            previous_dataFrame = current_dataFrame;
            previousTransformation = currentTransformation;
            std::cout << "Mesh written " << std::endl;
        }

        std::string frame_whole{ std::string("./mesh_without_raycasting/frame_whole_transformed_test") + std::string(".off") };
        meshZero.writeMesh(frame_whole);
    }
} // namespace kinect_fusion


