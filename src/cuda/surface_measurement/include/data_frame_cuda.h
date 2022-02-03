#pragma once

#include <iostream>

#include <type_definitions_cuda.h>
#include <Eigen.h>
#include <surface_cuda.h>

namespace kinect_fusion {

class FrameDataCuda {
    public:
        FrameDataCuda() {};
        /**
         * @brief Construct a new Frame Data object
         * 
         * @param[in] cameraIntrinstics Camera instinstics, or K matrix
         * @param[in] height Camera height, i.e. number of rows
         * @param[in] width Camera width, i.e. number of columns
         */
        FrameDataCuda(const Eigen::Matrix3f& cameraIntrinstics, std::size_t height, std::size_t width);

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
        SurfaceCuda& getSurface(Level level = 0U) {
            return m_surfaces[level];
        }

        /**
         * @brief Get the filtered depths values.
         */
        Map2DfCuda& getFilteredDepths(Level level = 0U) {
            return m_filteredDepthMaps[level];
        }

        Eigen::Matrix3f& getCameraIntrinsics(Level level)
        {
            return m_cameraIntrinstics[static_cast<std::size_t>(level)];
        }

        void printDataFrame()
        {
            for (int i = 0; i != NUMBER_OF_LEVELS; i++)
            {
                Map2DVector3fCuda normal_map = m_surfaces[i].getNormalMap();
                Map2DVector3fCuda vertex_map = m_surfaces[i].getVertexMap();

                for (int j = 0; j != normal_map.size(); j++)
                {
                    std::cout << " vertetx "  << vertex_map.get(j)<<std::endl;
                    std::cout << " normal " << normal_map.get(j) << std::endl;

                }
            }
        }
    private:
        /**
         * @brief Row depth map from sensor.
         */
        Map2DfCuda m_rowDepthMap;

        /**
         * @brief Filtered depth maps.
         */
        std::array<Map2DfCuda, NUMBER_OF_LEVELS> m_filteredDepthMaps;

        /**
         * @brief Vertex and normal maps for different levels (i.e. V matrices).
         */
        std::array<SurfaceCuda, NUMBER_OF_LEVELS> m_surfaces;
        
        /**
         * @brief Camera instrinstics for different levels, i.e. K.
         * 
         * @note Since the images on different levels have different resolutions, camera instrinstics change accordingly.
         */
        std::array<Eigen::Matrix3f, NUMBER_OF_LEVELS> m_cameraIntrinstics;    
};
} // namespace kinect_fusion