#include <virtual_sensor.h>

namespace kinect_fusion {

int main(int argc, char *argv[]) {
    std::string filenameIn{"../data/rgbd_dataset_freiburg1_xyz/"};

    VirtualSensor sensor;
    sensor.init(filenameIn)

    FrameData dataFrame{sensor.getDepthIntrinsics(), sensor.getHeight(), sensor.getWidth()};

    sensor.processNextFrame();
    Map2Df depth = sensor.getDepth();
    dataFrame.updateValues(depth);

    Surface& previousSurface = dataFrame.getSurface();

    for (unsigned int i = 1; i < sensor.getCurrentFrameCnt(); i++) {
        sensor.processNextFrame();

        Map2Df depth = sensor.getDepth();
        dataFrame.updateValues(depth);

        Surface& nextSurface = dataFrame.getSurface();
    }
}
} // namespace kinect_fusion