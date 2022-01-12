#include <virtual_sensor.h>
#include "utils/include/type_definitions.h"
#include <PoseEstimator.h>
#include <data_frame.h>
namespace kinect_fusion {

int main(int argc, char *argv[]) {
    std::string filenameIn{"../data/rgbd_dataset_freiburg1_xyz/"};

    VirtualSensor sensor;
    sensor.init(filenameIn)

    FrameData previous_dataFrame{sensor.getDepthIntrinsics(), sensor.getHeight(), sensor.getWidth()};

    sensor.processNextFrame();
    Map2Df depth = sensor.getDepth();
    dataFrame.updateValues(depth);

    Surface& previousSurface = dataFrame.getSurface();
    MatrixXf& pose_estimation = MatrixXf::Identity();
    for (unsigned int i = 1; i < sensor.getCurrentFrameCnt(); i++) {
        sensor.processNextFrame();
        FrameData current_dataFrame{ sensor.getDepthIntrinsics(), sensor.getHeight(), sensor.getWidth() };
        Map2Df depth = sensor.getDepth();
        dataFrame.updateValues(depth);

        Surface& nextSurface = dataFrame.getSurface();

        PoseEstimator pose_estimator(current_dataFrame, previous_dataFrame, pose_estimation);
        
        for (Level level:  LEVELS)
        {
            frame2frameEstimation(Eigen::MatrixXf inputTransformationMatrix, level);
        }
        pose_estimation = pose_estimator.getCurrentTransformation();
    }
}
} // namespace kinect_fusion