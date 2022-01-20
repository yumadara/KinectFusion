#include <data_frame.h>

#include <surface_measurement_utils.h>

#include <iostream>

namespace kinect_fusion {

FrameData::FrameData(const Eigen::Matrix3f& cameraIntrinstics, std::size_t height, std::size_t width)
    {
        for (Level level = 0U; level < NUMBER_OF_LEVELS; level++) {
            
            std::cout << "Level index: " << level << std::endl;

            std::size_t scale = std::pow(2U, level);

            std::size_t current_height = height / scale;
            std::size_t current_width = width / scale;

            m_cameraIntrinstics[level] = computeLevelCameraIntrinstics(cameraIntrinstics, level);

            m_filteredDepthMaps[level] = Map2Df(current_height, current_width);
            m_surfaces[level] = Surface(current_height, current_width);
            
            std::cout << "Surface height: " << m_surfaces[level].getHeight() << std::endl;
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

} // namespace kinect_fusion