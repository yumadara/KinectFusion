#include <fstream> 

#include <virtual_sensor.h>
#include <type_definitions.h>
#include <data_frame.h>
#include <simple_mesh.h>
#include "Sensor_Pose_Estimation/include/PoseEstimator.h"

using namespace kinect_fusion;

int main(int argc, char *argv[]) {
    std::string filenameIn{"./data/rgbd_dataset_freiburg1_xyz/"};

    VirtualSensor sensor;
    sensor.init(filenameIn);

    FrameData currentDataFrame{sensor.getDepthIntrinsics(), sensor.getDepthImageHeight(), sensor.getDepthImageWidth()};
    FrameData previousDataFrame{sensor.getDepthIntrinsics(), sensor.getDepthImageHeight(), sensor.getDepthImageWidth()};

    MatrixXf previousTransformation = Matrix4f::Identity();
    MatrixXf currentTransformation = Matrix4f::Identity();

    sensor.processNextFrame();
    previousDataFrame.updateValues(sensor.getDepth());

    std::string frameZero{ std::string("./executables/generated_data/merged_meshes/frame_0_not_transformed") + std::string(".off") };
    SimpleMesh meshZero{previousDataFrame.getSurface().getVertexMap(), currentTransformation, 0.1f};
    meshZero.writeMesh(frameZero);

    for (unsigned int i = 1; i < 797; i++) {
        sensor.processNextFrame();

        Map2Df& depths{sensor.getDepth()};
        currentDataFrame.updateValues(depths);

        PoseEstimator pose_estimator(currentDataFrame, previousDataFrame, previousTransformation, currentTransformation, sensor);
        currentTransformation = pose_estimator.frame2frameEstimation(previousTransformation);
        std::cout << "Current transformation:\n" << currentTransformation << std::endl;

        std::string frameK{std::string("./executables/generated_data/merged_meshes/frame_") + std::to_string(i) + std::string(".off") };
        SimpleMesh meshK{currentDataFrame.getSurface().getVertexMap(), currentTransformation, 0.1f };
        meshK.writeMesh(frameK);
        // meshZero = SimpleMesh::joinMeshes(meshZero, meshK);
        std::cout << "Mesh written " << std::endl;

        previousTransformation = currentTransformation;
        previousDataFrame = currentDataFrame;
    }
    return 0;
}
// } // namespace kinect_fusion