#include <virtual_sensor.h>
#include <type_definitions.h>
#include <pose_estimator.h>
#include <data_frame.h>
#include <surface_reconstruct.h>
#include <ray_casting.h>
#include <camera.h>

using namespace kinect_fusion;

const std::string DATASET_PATH = "./data/rgbd_dataset_freiburg1_xyz/";
const std::string OUTPUT_PREFIX = "./executables/generated_data";

constexpr bool FRAME_TO_MODEL = false; // It does not work because raycasting does not provide enough points

int main(int argc, char *argv[]) {
    // Initialization
    VirtualSensor sensor;
    sensor.init(DATASET_PATH);

    Voxel volum(200, 200, 300, 100, 100, 0, -100, 0);
    
    FrameData previousDataFrame{sensor.getDepthIntrinsics(), sensor.getDepthImageHeight(), sensor.getDepthImageWidth() };
    FrameData currentDataFrame{ sensor.getDepthIntrinsics(), sensor.getDepthImageHeight(), sensor.getDepthImageWidth() };

    MatrixXf previousTransformation = Matrix4f::Identity();
    MatrixXf currentTransformation = Matrix4f::Identity();

    // Process first frame
    // Surface measurement
    sensor.processNextFrame();
    Map2Df depth = sensor.getDepth();
    previousDataFrame.updateValues(depth);
    std::cout << "Surface Measurement completed... " << std::flush;

    // Surface reconstruction
    update_volument(sensor, volum, previousTransformation);
    std::cout << "Surface reconstructed... " << std::endl;

    // Render first frame (without raycasting yet)
    std::string frameZero{OUTPUT_PREFIX + "/merged_meshes/frame_0_not_transformed.off"};
    SimpleMesh meshZero{previousDataFrame.getSurface().getVertexMap(), currentTransformation, 0.1f};
    meshZero.writeMesh(frameZero);
    std::cout << "Mesh written!" << std::endl;
    
    for (unsigned int i = 1; i < sensor.getNumberOfFrames(); i++) {
        // Surface measurement
        sensor.processNextFrame();
        Map2Df& depth = sensor.getDepth();
        currentDataFrame.updateValues(depth);
        std::cout << "Surface Measurement completed... " << std::flush;
       
        assert(previousTransformation(2, 3) == currentTransformation(2, 3));

        // Pose estimation
        PoseEstimator pose_estimator(currentDataFrame, previousDataFrame, previousTransformation, currentTransformation, sensor);
        currentTransformation = pose_estimator.frame2frameEstimation(previousTransformation);
        std::cout << "Pose estimation obtained... " << std::flush;

        // Surface reconstruction
        update_volument(sensor, volum, currentTransformation);
        Camera camera(currentTransformation, sensor.getDepthIntrinsics(),
            sensor.getDepthImageHeight(), sensor.getDepthImageWidth(),
            0, 0);
        std::cout << "Surface reconstructed... " << std::flush;

        // Raycasting
        RayCasting cast(volum, camera, depth);
        cast.do_work();
        Map2Df depthMap = cast.getDepthMap();
        previousDataFrame.updateValues(depthMap);
        std::cout << "Raycasting performed... " << std::flush;

        // Render raycasting results
        std::string frameK{OUTPUT_PREFIX + std::string("/merged_meshes/frame_") + std::to_string(i) + std::string(".off") };
        SimpleMesh meshK{currentDataFrame.getSurface().getVertexMap(), currentTransformation, 0.1f };
        meshK.writeMesh(frameK);
        std::cout << "Mesh written!" << std::endl;

        // Save previous data frame and transformation
        if (!FRAME_TO_MODEL) {
            previousDataFrame = currentDataFrame;
        }
        previousTransformation = currentTransformation;
    }
}
 // namespace kinect_fusion