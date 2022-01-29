#include <virtual_sensor.h>
#include "utils/include/type_definitions.h"
#include "Sensor_Pose_Estimation/include/PoseEstimator.h"
#include <data_frame.h>
#include"surface_reconstruct.h"
#include <RayCasting.h>
#include <Camera.h>

using namespace kinect_fusion;

int main(int argc, char *argv[]) {
    std::string filenameIn{ "/mnt/d/Users/chiyu/1 semester/KinectFusion/data/rgbd_dataset_freiburg1_xyz/" };
    std::string filenameBaseOut = std::string("./mesh/mesh_");
    VirtualSensor sensor;
    sensor.init(filenameIn);
    Voxel volum(200, 200, 300, 100, 100, 0, -100, 0);
    FrameData previous_dataFrame{sensor.getDepthIntrinsics(), sensor.getDepthImageHeight(), sensor.getDepthImageWidth() };

    sensor.processNextFrame();
    Map2Df depth = sensor.getDepth();
    previous_dataFrame.updateValues(depth);

    MatrixXf previousTransformation = Matrix4f::Identity();
    MatrixXf currentTransformation = Matrix4f::Identity();
    update_volument(sensor, volum, previousTransformation.inverse());

    /*std::string frameZero{ std::string("./mesh/""frame_0_not_transformed") + std::string(".off") };
    SimpleMesh meshZero{ previous_dataFrame.getSurface().getVertexMap(), previousTransformation, 0.1f };
    meshZero.writeMesh(frameZero);*/
    //
    //std::stringstream ss_first;
    //ss_first << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off";
    //std::cout << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off" << std::endl;
    //SimpleMesh lastDepthMesh{ sensor, previousTransformation, 0.1f };
    //lastDepthMesh.writeMesh(ss_first.str());

    for (unsigned int i = 1; i < 798; i++) {
        //currentTransformation = 
        std::cout << "Now we are at frame " << i << std::endl;
        sensor.processNextFrame();
        FrameData current_dataFrame{ sensor.getDepthIntrinsics(), sensor.getDepthImageHeight(), sensor.getDepthImageWidth() };
        Map2Df depth = sensor.getDepth();
        current_dataFrame.updateValues(depth);

        //std::stringstream ss_current_frame_not_transformed;
        //ss_current_frame_not_transformed << filenameBaseOut << sensor.getCurrentFrameCnt() << "_not_transformed.off";
        //std::cout << filenameBaseOut << sensor.getCurrentFrameCnt() << "_not_transformed.off" << std::endl;
        //SimpleMesh currentDepthMeshNotTransformed{ sensor,previousTransformation , 0.1f };
        //currentDepthMeshNotTransformed.writeMesh(ss_current_frame_not_transformed.str());


        assert(previousTransformation(2, 3) == currentTransformation(2, 3));
        PoseEstimator pose_estimator(current_dataFrame, previous_dataFrame, previousTransformation, currentTransformation, sensor);

        currentTransformation = pose_estimator.frame2frameEstimation(previousTransformation);

        //std::stringstream ss_current_frame_transformed;
        //ss_current_frame_transformed << filenameBaseOut << sensor.getCurrentFrameCnt() << "_transformed.off";
        ////std::cout << filenameBaseOut << sensor.getCurrentFrameCnt() << "_not_transformed.off" << std::endl;
        //SimpleMesh currentDepthMeshTransformed{ sensor, currentTransformation, 0.1f };
        //currentDepthMeshTransformed.writeMesh(ss_current_frame_transformed.str());

        update_volument(sensor, volum, currentTransformation.inverse());

        Camera camera(currentTransformation, sensor.getDepthIntrinsics(), //TODO 2. sensor.getDepthIntrinsicsInverse
            sensor.getDepthImageHeight(), sensor.getDepthImageWidth(),
            0, 0);
        RayCasting cast(volum, camera);
        cast.do_work();
        Map2Df depthMap = cast.getDepthMap();  //TODO cast.getDepthMap
        previous_dataFrame.updateValues(depthMap);

        


        //std::stringstream ss_current_frame_transformed_updated;
        //ss_current_frame_transformed_updated << filenameBaseOut << sensor.getCurrentFrameCnt() << "_transformed_updated.off";
        ////std::cout << filenameBaseOut << sensor.getCurrentFrameCnt() << "_not_transformed.off" << std::endl;
        //SimpleMesh currentDepthMeshTransformedUpdated{ sensor, currentTransformation, 0.1f };
        //currentDepthMeshTransformedUpdated.writeMesh(ss_current_frame_transformed_updated.str());
        
        
   
        if (i % 5 == 0)
        {
            std::string frameK{ std::string("./mesh/frame_" + std::to_string(i) + "_transformed_test") + std::string(".off") };
            SimpleMesh meshK{ previous_dataFrame.getSurface().getVertexMap(), currentTransformation, 0.1f };
            meshK.writeMesh(frameK);
        }
        

        previousTransformation = currentTransformation;
        std::cout << "Mesh written " << std::endl;

    }
}
 // namespace kinect_fusion