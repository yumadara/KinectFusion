#pragma once

#include <iostream>

#include <Eigen.h>

#include <surface.h>

namespace kinect_fusion {

class FrameData {
    public:
        FrameData() {};
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
        void updateValues(Map2Df& depths);

        /**
         * @brief Get the surface with normal and vertex maps for the given level.
         * 
         * @return Surface& Wrapper around normal and vertex maps
         */
        Surface& getSurface(Level level = 0U) {
            return m_surfaces[level];
        }

        /**
         * @brief Get the filtered depths values.
         */
        Map2Df& getFilteredDepths(Level level = 0U) {
            return m_filteredDepthMaps[level];
        }

        /**
         * @brief Get the camera Intrinsics
         */
        Eigen::Matrix3f& getCameraIntrinsics(Level level = 0U)
        {
            return m_cameraIntrinstics[static_cast<std::size_t>(level)];
        }

        /**
         * @brief Print the vertex and normal maps for 3 levels for debugging.
         */
        void printDataFrame();

    private:
        /**
         * @brief Row depth map from sensor.
         */
        Map2Df m_rowDepthMap;

        /**
         * @brief Filtered depth maps.
         */
        std::array<Map2Df, NUMBER_OF_LEVELS> m_filteredDepthMaps;

        /**
         * @brief Vertex and normal maps for different levels (i.e. V matrices).
         */
        std::array<Surface, NUMBER_OF_LEVELS> m_surfaces;
        
        /**
         * @brief Camera instrinstics for different levels, i.e. K.
         * 
         * @note Since the images on different levels have different resolutions, camera instrinstics change accordingly.
         */
        std::array<Eigen::Matrix3f, NUMBER_OF_LEVELS> m_cameraIntrinstics;    
};
} // namespace kinect_fusion