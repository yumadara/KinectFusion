#include <fstream> 

#include <virtual_sensor.h>
#include <type_definitions.h>
#include <data_frame.h>

using namespace kinect_fusion;

int main(int argc, char *argv[]) {
    std::string filenameIn{"./data/rgbd_dataset_freiburg1_xyz/"};

    VirtualSensor sensor;
    sensor.init(filenameIn);
    FrameData dataFrame{sensor.getDepthIntrinsics(), sensor.getDepthImageHeight(), sensor.getDepthImageWidth()};

    for (unsigned int i = 0; i < 10; i++) {
        sensor.processNextFrame();

        auto depth = sensor.getDepth();
        dataFrame.updateValues(depth);

        for (std::size_t level = 0; level < NUMBER_OF_LEVELS; level++) {
            std::string vertexMapFileName{std::string("./src/executables/temp/vertex_map_level_") + std::to_string(level) + std::string("_frame_") + std::to_string(i) + std::string(".csv")};
            std::string normalMapFileName{std::string("./src/executables/temp/normal_map_level_") + std::to_string(level) + std::string("_frame_") + std::to_string(i) + std::string(".csv")};

            std::ofstream vertexFile{vertexMapFileName};
            std::ofstream normalFile{normalMapFileName};

            Map2DVector3f normal_map{dataFrame.getSurface(level).getNormalMap()};
            Map2DVector3f vertex_map{dataFrame.getSurface(level).getVertexMap()};

            for (int j = 0; j != normal_map.size(); j++)
            {
                for (int coordinate = 0; coordinate < 3; coordinate++)
                {
                    // std::cout << vertex_map.get(j)(coordinate) << ";" << std::endl;
                    vertexFile << vertex_map.get(j)(coordinate) << ";";
                    normalFile << normal_map.get(j)(coordinate) << ";";
                }
                vertexFile << std::endl;
                normalFile << std::endl;
            }
        }
    }
    return 0;
}
// } // namespace kinect_fusion