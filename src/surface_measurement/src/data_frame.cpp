#include <data_frame.h>

#include <surface_measurement_utils.h>

#include <iostream>

namespace kinect_fusion {

FrameData::FrameData(const Eigen::Matrix3f& cameraIntrinstics, std::size_t height, std::size_t width)
    {
        for (Level level = 0U; level < NUMBER_OF_LEVELS; level++) {
            std::size_t scale = std::pow(2U, level);

            std::size_t current_height = height / scale;
            std::size_t current_width = width / scale;

            m_cameraIntrinstics[level] = computeLevelCameraIntrinstics(cameraIntrinstics, level);

            m_filteredDepthMaps[level] = Map2Df(current_height, current_width);
            m_surfaces[level] = Surface(current_height, current_width);
        } 
    }

    void FrameData::updateValues(Map2Df& depths) {
        m_rowDepthMap = depths;
        
        applyBiliteralFilter(depths, m_filteredDepthMaps[0]); // Do not filter now
        // m_filteredDepthMaps[0] = depths;
        for (std::size_t i = 0; i < NUMBER_OF_LEVELS - 1; i++) {
            subsample(m_filteredDepthMaps[i], m_filteredDepthMaps[i + 1]);
        }
        for (std::size_t i = 0; i < NUMBER_OF_LEVELS; i++) {
            fillVertexMap(m_filteredDepthMaps[i], m_cameraIntrinstics[i], m_surfaces[i].getVertexMap());
            fillNormalMap(m_surfaces[i].getVertexMap(), m_surfaces[i].getNormalMap());
        }
    }

    void FrameData::printDataFrame()
        {
            for (int i = 0; i != NUMBER_OF_LEVELS; i++)
            {
                std::cout << "Level: " << (i + 1) << std::endl;
                Map2DVector3f normal_map = m_surfaces[i].getNormalMap();
                Map2DVector3f vertex_map = m_surfaces[i].getVertexMap();

                for (int j = 0; j != normal_map.size(); j++)
                {
                    std::cout << "Vertex "  << vertex_map.get(j) << std::endl;
                    std::cout << "Normal " << normal_map.get(j) << std::endl;
                }
            }
        }

} // namespace kinect_fusion