#include <data_frame.h>

#include <surface_measurement_utils.h>

namespace kinect_fusion {

FrameData::FrameData(const Eigen::Matrix3f& cameraIntrinstics, std::size_t height, std::size_t width)
    {
        for (Level level : LEVELS) {
            std::size_t index = getIndex(level);
            std::size_t scale = std::pow(2U, index);

            std::size_t current_height = height / scale;
            std::size_t current_width = width / scale;

            m_cameraIntrinstics[index] = computeLevelCameraIntrinstics(cameraIntrinstics, level);

            m_normalMaps[index] = Map2DVector3f(current_height, current_width);
            m_vertexMaps[index] = Map2DVector3f(current_height, current_width);
            m_filteredDepthMaps[index] = Map2Df(current_height, current_width);
        } 
    }

    void FrameData::updateValues(const Map2Df&  depths) {
        m_rowDepthMap = depths;

        m_filteredDepthMaps[0] = depths;
        for (std::size_t i = 0; i < NUMBER_OF_LEVELS - 1; i++) {
            subsample(m_filteredDepthMaps[i], m_filteredDepthMaps[i + 1]);
        }
        for (std::size_t i = 0; i < NUMBER_OF_LEVELS; i++) {
            fillVertexMap(m_filteredDepthMaps[i], m_cameraIntrinstics[i], m_vertexMaps[i]);
            fillNormalMap(m_vertexMaps[i], m_normalMaps[i]);
        }
    }

} // namespace kinect_fusion