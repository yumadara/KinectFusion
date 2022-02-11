#include <virtual_sensor.h>
#include "utils/include/type_definitions.h"
#include "Sensor_Pose_Estimation/include/PoseEstimator.h"
#include <data_frame.h>
#include"surface_reconstruct.h"
#include <RayCasting.h>
#include <Camera.h>

using namespace kinect_fusion;

int main(int argc, char *argv[]) {
    std::string filenameIn{ "./data/rgbd_dataset_freiburg1_xyz/" };
    std::string filenameBaseOut = std::string("./mesh/mesh_");
    VirtualSensor sensor;
    sensor.init(filenameIn);
    Voxel volum(200, 200, 300, 100, 100, 0, -100, 0);
    FrameData previous_dataFrame{sensor.getDepthIntrinsics(), sensor.getDepthImageHeight(), sensor.getDepthImageWidth() };

    sensor.processNextFrame();
    Map2Df depth = sensor.getDepth();
    previous_dataFrame.updateValues(depth);
    std::cout << "first frame" << std::endl;

    MatrixXf previousTransformation = Matrix4f::Identity();
    MatrixXf currentTransformation = Matrix4f::Identity();
    update_volument(sensor, volum, previousTransformation);
    std::cout << "volument updated" << std::endl;
    
    std::string frameZero{ std::string("./mesh/""frame_0_not_transformed") + std::string(".off") };
    SimpleMesh meshZero{ previous_dataFrame.getSurface().getVertexMap(), previousTransformation, 0.1f };
    meshZero.writeMesh(frameZero);
    

    for (unsigned int i = 1; i < 4; i++) {
        //currentTransformation = 
        std::cout << "Now we are at frame " << i << std::endl;
        sensor.processNextFrame();
        FrameData current_dataFrame{ sensor.getDepthIntrinsics(), sensor.getDepthImageHeight(), sensor.getDepthImageWidth() };
        Map2Df depth = sensor.getDepth();
        current_dataFrame.updateValues(depth);
       
        assert(previousTransformation(2, 3) == currentTransformation(2, 3));
        PoseEstimator pose_estimator(current_dataFrame, previous_dataFrame, previousTransformation, currentTransformation, sensor);

        currentTransformation = pose_estimator.frame2frameEstimation(previousTransformation);
        
        update_volument(sensor, volum, currentTransformation);

        Camera camera(currentTransformation, sensor.getDepthIntrinsics(), //TODO 2. sensor.getDepthIntrinsicsInverse
            sensor.getDepthImageHeight(), sensor.getDepthImageWidth(),
            0, 0);

        RayCasting cast(volum, camera, depth);
        cast.do_work();
        Map2Df depthMap = cast.getDepthMap();  //TODO cast.getDepthMap
        previous_dataFrame.updateValues(depthMap);
        
            std::string frameK{ std::string("./mesh/frame_" + std::to_string(i) + "_transformed_test") + std::string(".off") };
            SimpleMesh meshK{ previous_dataFrame.getSurface().getVertexMap(), currentTransformation, 0.1f };
            meshK.writeMesh(frameK);
            meshZero = SimpleMesh::joinMeshes(meshZero, meshK);
        
        //previous_dataFrame = current_dataFrame;
        previousTransformation = currentTransformation;
        std::cout << "Mesh written " << std::endl;

        
    }
    std::string frame_whole{ std::string("./mesh/frame_whole_transformed_test") + std::string(".off") };
    meshZero.writeMesh(frame_whole);
}
 // namespace kinect_fusion