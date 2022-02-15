#include <fstream> 

#include <virtual_sensor.h>
#include <type_definitions.h>
#include <data_frame.h>
#include <simple_mesh.h>

using namespace kinect_fusion;

const std::string DATASET_PATH = "./data/rgbd_dataset_freiburg1_xyz/";
const std::string OUTPUT_PREFIX = "./executables/generated_data/surface_measurement";

int main(int argc, char *argv[]) {
    VirtualSensor sensor;
    sensor.init(DATASET_PATH);
    
    FrameData dataFrame{sensor.getDepthIntrinsics(), sensor.getDepthImageHeight(), sensor.getDepthImageWidth()};

    for (unsigned int i = 0; i < 10; i++) {
        sensor.processNextFrame();

        Map2Df& depths{sensor.getDepth()};
        dataFrame.updateValues(depths);

        std::ofstream depthsFile{OUTPUT_PREFIX + std::string("/depths_csv/frame_") + std::to_string(i) + std::string(".csv")};
        std::size_t last_column = depths.getNumberOfColumns() - 1;
        for (std::size_t row = 0; row < depths.getNumberOfRows(); row++) {
            for (std::size_t col = 0; col < last_column; col++) {
                depthsFile << depths.get(row, col) << ";";
            }
            depthsFile << depths.get(row, last_column) << "\n";
        }

        std::ofstream filteredDepthsFile{OUTPUT_PREFIX + std::string("/depths_csv/filtered_frame_") + std::to_string(i) + std::string(".csv")};
        last_column = depths.getNumberOfColumns() - 1;
        for (std::size_t row = 0; row < depths.getNumberOfRows(); row++) {
            for (std::size_t col = 0; col < last_column; col++) {
                filteredDepthsFile << dataFrame.getFilteredDepths().get(row, col) << ";";
            }
            filteredDepthsFile << dataFrame.getFilteredDepths().get(row, last_column) << "\n";
        }

        std::string unfilteredMeshFile{OUTPUT_PREFIX + std::string("/meshes/unfiltered_frame_") + std::to_string(i) + std::string(".off")};
        SimpleMesh unfilteredMesh{sensor, Matrix4f::Identity(), 0.1f};
		unfilteredMesh.writeMesh(unfilteredMeshFile);
        
        for (std::size_t level = 0; level < NUMBER_OF_LEVELS; level++) {
            std::string meshFileName{OUTPUT_PREFIX + std::string("/meshes/frame_") + std::to_string(i) + std::string("_level_") + std::to_string(level) + std::string(".off")};
		    SimpleMesh depthMesh{dataFrame.getSurface(level).getVertexMap(), Matrix4f::Identity(), 0.1f};
		    depthMesh.writeMesh(meshFileName);

            std::ofstream vertexFile{OUTPUT_PREFIX + std::string("/vertices_csv/frame_") + std::to_string(i) + std::string("_level_") + std::to_string(level) + std::string(".csv")};
            std::ofstream normalFile{OUTPUT_PREFIX + std::string("/normals_csv/frame_") + std::to_string(i) + std::string("_level_") + std::to_string(level) + std::string(".csv")};

            Map2DVector3f& normal_map{dataFrame.getSurface(level).getNormalMap()};
            Map2DVector3f& vertex_map{dataFrame.getSurface(level).getVertexMap()};

            for (int j = 0; j != normal_map.size(); j++)
            {
                for (int coordinate = 0; coordinate < 3; coordinate++)
                {
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