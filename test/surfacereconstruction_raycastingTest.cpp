#include <virtual_sensor.h>
#include "utils/include/type_definitions.h"
#include <data_frame.h>
#include"surface_reconstruct.h"
#include <simple_mesh.h>
#include <Camera.h>
#include <RayCasting.h>
#include <gtest/gtest.h>
namespace kinect_fusion {


    TEST(TSDFRaycastTest, checkCombined) {
    std::string filenameIn{"../data/rgbd_dataset_freiburg1_xyz/"};

    VirtualSensor sensor;
    sensor.init(filenameIn);
    // Voxel volum(600,600,600, 300, 300, 300, -100, 0);
    Voxel volum(200,200,300,100,100,0, -100, 0);
    // FrameData previous_dataFrame{sensor.getDepthIntrinsics(), sensor.getHeight(), sensor.getWidth()};

    sensor.processNextFrame();
    // Map2Df depth = sensor.getDepth();
    // dataFrame.updateValues(depth);

    
    // MatrixXf& pose_estimation = MatrixXf::Identity();
    for (unsigned int i = 1; i < 2; i++) {
        
        sensor.processNextFrame();
        SimpleMesh origDepthMesh{ sensor, Matrix4f::Identity(), 0.1f };
        std::stringstream so;
        so << "raycastingInput" << sensor.getCurrentFrameCnt() << ".off";
        origDepthMesh.writeMesh(so.str());
        // FrameData current_dataFrame{ sensor.getDepthIntrinsics(), sensor.getHeight(), sensor.getWidth() };
        // depth = sensor.getDepth();
        // dataFrame.updateValues(depth);

        

        // PoseEstimator pose_estimator(current_dataFrame, previous_dataFrame, pose_estimation);
        
        // for (Level level:  LEVELS)
        // {
        //     frame2frameEstimation(pose_estimation, level);
        // }
        // pose_estimation = pose_estimator.getCurrentTransformation();

        update_volument(sensor, volum, Matrix4f::Identity() );
        // Camera camera(pose_estimation, sensor.getDepthIntrinsicsInverse(), //TODO 2. sensor.getDepthIntrinsicsInverse
        //     sensor.getHeight(), sensor.getWidth(),
        //     0, 0);
        // RayCasting cast(volum, camera);
        // cast.do_work();
        // Map2Df depthMap = cast.getDepthMap();  //TODO cast.getDepthMap
        
        // previous_dataFrame.updateValues(depthMap::Identity());
        

    }
    Camera camera(Matrix4f::Identity(), sensor.getDepthIntrinsics(), //TODO 2. sensor.getDepthIntrinsicsInverse
        sensor.getDepthImageHeight(), sensor.getDepthImageWidth(),
        0, 0);
    RayCasting cast(volum, camera);
    cast.do_work();
    Map2Df depthMap = cast.getDepthMap(); 
    sensor.setDepth(depthMap);
    SimpleMesh currentDepthMesh{ sensor, Matrix4f::Identity(), 0.1f };
    std::stringstream ss;
	ss << "raycastingOutput" << sensor.getCurrentFrameCnt() << ".off";
    currentDepthMesh.writeMesh(ss.str());



    // return 0;
    }


} 