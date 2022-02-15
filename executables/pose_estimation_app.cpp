#include <fstream> 

#include <virtual_sensor.h>
#include <type_definitions.h>
#include <data_frame.h>
#include <simple_mesh.h>
#include <pose_estimator.h>

using namespace kinect_fusion;

const std::string DATASET_PATH = "./data/rgbd_dataset_freiburg1_xyz/";
const std::string OUTPUT_PREFIX = "./executables/generated_data";

int main(int argc, char *argv[]) {
    std::string filenameIn{DATASET_PATH};

    VirtualSensor sensor;
    sensor.init(filenameIn);

    FrameData currentDataFrame{sensor.getDepthIntrinsics(), sensor.getDepthImageHeight(), sensor.getDepthImageWidth()};
    FrameData previousDataFrame{sensor.getDepthIntrinsics(), sensor.getDepthImageHeight(), sensor.getDepthImageWidth()};

    MatrixXf previousTransformation = Matrix4f::Identity();
    MatrixXf currentTransformation = Matrix4f::Identity();

    sensor.processNextFrame();
    previousDataFrame.updateValues(sensor.getDepth());

    std::string frameZero{OUTPUT_PREFIX + "/merged_meshes/frame_0_not_transformed.off"};
    SimpleMesh meshZero{previousDataFrame.getSurface().getVertexMap(), currentTransformation, 0.1f};
    meshZero.writeMesh(frameZero);

    for (unsigned int i = 1; i < 797; i++) {
        sensor.processNextFrame();

        Map2Df& depths{sensor.getDepth()};
        currentDataFrame.updateValues(depths);
        std::cout << "Surface measurement completed... " << std::flush;

        PoseEstimator pose_estimator(currentDataFrame, previousDataFrame, previousTransformation, currentTransformation, sensor);
        currentTransformation = pose_estimator.frame2frameEstimation(previousTransformation);
        std::cout << "Pose estimation obtained... " << std::flush;

        std::string frameK{OUTPUT_PREFIX + std::string("/merged_meshes/frame_") + std::to_string(i) + std::string(".off") };
        SimpleMesh meshK{currentDataFrame.getSurface().getVertexMap(), currentTransformation, 0.1f };
        meshK.writeMesh(frameK);
        std::cout << "Mesh written!" << std::endl;

        previousTransformation = currentTransformation;
        previousDataFrame = currentDataFrame;
    }
    return 0;
}
// } // namespace kinect_fusion