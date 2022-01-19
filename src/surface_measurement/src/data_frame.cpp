#include <data_frame.h>

#include <surface_measurement_utils.h>

#include <iostream>

namespace kinect_fusion {

FrameData::FrameData(const Eigen::Matrix3f& cameraIntrinstics, std::size_t height, std::size_t width)
    {
        for (Level level : LEVELS) {
            
            std::size_t index = getIndex(level);
            std::cout << "index " << index << std::endl;

            std::size_t scale = std::pow(2U, index);

            std::size_t current_height = height / scale;
            std::size_t current_width = width / scale;

            m_cameraIntrinstics[index] = computeLevelCameraIntrinstics(cameraIntrinstics, level);

            m_filteredDepthMaps[index] = Map2Df(current_height, current_width);
            m_surfaces[index] = Surface(current_height, current_width);
            
            std::cout << "surface height" << m_surfaces[index].getHeight() << std::endl;
        } 
    }

    void FrameData::updateValues(Map2Df& depths) {
        m_rowDepthMap = depths;
        
        // applyBiliteralFilter(depths, m_filteredDepthMaps[0]); // Do not filter now
        for (std::size_t i = 0; i < NUMBER_OF_LEVELS - 1; i++) {
            subsample(m_filteredDepthMaps[i], m_filteredDepthMaps[i + 1]);
        }
        for (std::size_t i = 0; i < NUMBER_OF_LEVELS; i++) {
            fillVertexMap(m_filteredDepthMaps[i], m_cameraIntrinstics[i], m_surfaces[i].getVertexMap());
            fillNormalMap(m_surfaces[i].getVertexMap(), m_surfaces[i].getNormalMap());
        }
    }

} // namespace kinect_fusion