#include <virtual_sensor.h>
#include "utils/include/type_definitions.h"
#include <PoseEstimator.h>
#include <data_frame.h>
#include"surface_reconstruct.h"
namespace kinect_fusion {

int main(int argc, char *argv[]) {
    std::string filenameIn{"../data/rgbd_dataset_freiburg1_xyz/"};

    VirtualSensor sensor;
    sensor.init(filenameIn)
    Voxel volum(100, 100, 100, 0, 0, 0, -100, 0);
    FrameData previous_dataFrame{sensor.getDepthIntrinsics(), sensor.getHeight(), sensor.getWidth()};

    sensor.processNextFrame();
    Map2Df depth = sensor.getDepth();
    dataFrame.updateValues(depth);

    
    MatrixXf& pose_estimation = MatrixXf::Identity();
    for (unsigned int i = 1; i < sensor.getCurrentFrameCnt(); i++) {
        sensor.processNextFrame();
        FrameData current_dataFrame{ sensor.getDepthIntrinsics(), sensor.getHeight(), sensor.getWidth() };
        Map2Df depth = sensor.getDepth();
        dataFrame.updateValues(depth);

        

        PoseEstimator pose_estimator(current_dataFrame, previous_dataFrame, pose_estimation);
        
        for (Level level:  LEVELS)
        {
            frame2frameEstimation(pose_estimation, level);
        }
        pose_estimation = pose_estimator.getCurrentTransformation();

        update_volument(sensor, volum, pose_estimation);
        Camera camera(pose_estimation, sensor.getDepthIntrinsicsInverse(), //TODO 1. camera:change constructor 2. sensor.getDepthIntrinsicsInverse
            sensor.getHeight(), sensor.getWidth(),
            0, 0);
        RayCasting cast(volum, camera);
        cast.do_work();
        Map2D depthMap = cast.getDepthMap();  //TODO cast.getDepthMap
        
        previous_dataFrame.updateValues(depthMap);
        

    }
}
} // namespace kinect_fusion