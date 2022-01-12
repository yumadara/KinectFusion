#pragma once

#include <Eigen.h>

#include <surface.h>

namespace kinect_fusion {

constexpr float SIGMA{0.1};

class FrameData {
    public:
        /**
         * @brief Construct a new Frame Data object
         * 
         * @param[in] cameraIntrinstics Camera instinstics, or K matrix
         * @param[in] height Camera height, i.e. number of rows
         * @param[in] width Camera width, i.e. number of columns
         */
        FrameData(const Eigen::Matrix3f& cameraIntrinstics, std::size_t height, std::size_t width);

        /**
         * @brief Update frame data values with new depths map.
         * 
         * @param depths Depths map from depth camera
         */
        void updateValues(const Map2Df& depths);

        /**
         * @brief Get the surface with normal and vertex maps for the first level.
         * 
         * @return Surface& Wrapper around normal and vertex maps
         */
        Surface& getSurface() {
            return m_surfaces[0];
        }

        /**
         * @brief Get the surface with normal and vertex maps for the given level.
         * 
         * @return Surface& Wrapper around normal and vertex maps
         */
        Surface& getSurface(Level level) {
            return m_surfaces[getIndex(level)];
        }

        Eigen::Matrix3f& getCameraIntrinsics(Level level)
        {
            return m_cameraIntrinstics[getIndex(level)];
        }


    private:
        /**
         * @brief Get index of m_filteredDepthMaps, m_normalMaps, or other maps in this class, from the level.
         * 
         * @param level Level, as described in KinectFusion paper
         * @return std::size_t index of corresponding maps
         */
        inline std::size_t getIndex(Level level) {
            return static_cast<std::size_t>(level) - 1U;
        }

        /**
         * @brief Row depth map from sensor
         */
        Map2Df m_rowDepthMap;


        Map2Df m_filteredDepthMaps[NUMBER_OF_LEVELS];

        /**
         * @brief Vertex and normal maps for different levels (i.e. V matrices).
         */
        Surface m_surfaces[NUMBER_OF_LEVELS];
        
        /**
         * @brief Camera instrinstics for different levels, i.e. K.
         * 
         * @note Since the images on different levels have different resolutions, camera instrinstics change accordingly.
         */
        Eigen::Matrix3f m_cameraIntrinstics[NUMBER_OF_LEVELS];          
};
} // namespace kinect_fusion