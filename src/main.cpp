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

    VirtualSensor sensor;
    sensor.init(filenameIn);
    Voxel volum(100, 100, 100, 0, 0, 0, -100, 0);
    FrameData previous_dataFrame{sensor.getDepthIntrinsics(), sensor.getDepthImageHeight(), sensor.getDepthImageWidth() };

    sensor.processNextFrame();
    Map2Df depth = sensor.getDepth();
    previous_dataFrame.updateValues(depth);

    
    MatrixXf previousTransformation = Matrix4f::Identity();
    MatrixXf currentTransformation = Matrix4f::Identity();

    for (unsigned int i = 1; i < 3; i++) {
        //currentTransformation = 
        sensor.processNextFrame();
        FrameData current_dataFrame{ sensor.getDepthIntrinsics(), sensor.getDepthImageHeight(), sensor.getDepthImageWidth() };
        Map2Df depth = sensor.getDepth();
        current_dataFrame.updateValues(depth);

        PoseEstimator pose_estimator(current_dataFrame, previous_dataFrame, previousTransformation, currentTransformation, sensor);

        currentTransformation = pose_estimator.frame2frameEstimation(previousTransformation);

        update_volument(sensor, volum, currentTransformation);
        Camera camera(currentTransformation, sensor.getDepthIntrinsics(), //TODO 2. sensor.getDepthIntrinsicsInverse
            sensor.getDepthImageHeight(), sensor.getDepthImageWidth(),
            0, 0);
        RayCasting cast(volum, camera);
        cast.do_work();
        Map2Df depthMap = cast.getDepthMap();  //TODO cast.getDepthMap
        previous_dataFrame.updateValues(depthMap);

        std::string meshFileName{ std::string("./mesh/") + std::to_string(i) + std::string("_frame_") +  std::string(".off") };
        SimpleMesh depthMesh{ previous_dataFrame.getSurface().getVertexMap(), currentTransformation, 0.1f };
        depthMesh.writeMesh(meshFileName);
        previousTransformation = currentTransformation;
        std::cout << "Mesh written " << std::endl;

    }
}
 // namespace kinect_fusion