#pragma once

#include <Eigen.h>

#include <type_definitions.h>

namespace kinect_fusion {

    constexpr std::size_t NUMBER_OF_LEVELS = 3;

    constexpr float SIGMA{0.1};

    /**
     * @brief Levels.
     */
    enum class Level : std::size_t {
        First = 1U,
        Second = 2U,
        Third = 3U
    };

    constexpr Level LEVELS[NUMBER_OF_LEVELS] {
        Level::First, Level::Second, Level::Third
    };

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

        private:
            /**
             * @brief Fill vertex map given depths values and camera instinsics using
             * VertexMap(row, column) = DepthMap(row, column) * K^(-1) * (row, column, 1);
             * 
             * @param[in] depths Depth map
             * @param[in] cameraIntrinsics Camera intrinsics, or K matrix
             * @param[out] vertexMap Vertex map that should be filled
             */
            static void fillVertexMap(const Map2Df& depths, const Eigen::Matrix3f& cameraIntrinsics, Map2DVector3f& vertexMap);

            /**
             * @brief Fill normal map given vertex map (V) using
             * NormalMap(row, col) = (V(row + 1, col) − V(row, col)) × (V(row, col + 1) - V(row, col))
             * NormalMap(row, col) = NormalMap(row, col) / || NormalMap(row, col) ||
             * 
             * @param[in] vertexMap Vertex map
             * @param[out] normalMap Normal Map, which should be filled
             */
            static void fillNormalMap(const Map2DVector3f& vertexMap, Map2DVector3f& normalMap);

            /**
             * @brief Fill next level depth map from previous depth map by block averaging pixels values and
             * subsampling with halve resolution.
             * 
             * @note nextDepthMap should have 2 times smaller dimensions than previousDepthMap.
             * @note While block averaging, if some block pixel value is too far away from central pixel, 
             * it will be ignored.
             * 
             * @param[in] previousDepthMap Depth map which should be subsampled
             * @param[out] nextDepthMap Depth map in which to subsampled depth map will be written.
             */
            static void subsample(const Map2Df& previousDepthMap, Map2Df& nextDepthMap);

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
             * @brief Vertex maps for different levels (i.e. V matrices).
             */
            Map2DVector3f m_vertexMaps[NUMBER_OF_LEVELS];

            /**
             * @brief Normal maps based on vertex maps for different levels (i.e. N matrices).
             */
            Map2DVector3f m_normalMaps[NUMBER_OF_LEVELS];
            
            /**
             * @brief Camera instrinstics for different levels, i.e. K.
             * 
             * @note Since the images on different levels have different resolutions, camera instrinstics change accordingly.
             */
            Eigen::Matrix3f m_cameraIntrinstics[NUMBER_OF_LEVELS];

            /**
             * @brief Get the Camera Intrinstics for specific level.
             * 
             * @param originalCameraIntrinstics Camera Intrinstics for level 0, or real camera intrinstics
             * @param level for which camera intrinstics should be computed
             * @return Eigen::Matrix3f camera intrinstics for this elvel.
             */
            static Eigen::Matrix3f computeLevelCameraIntrinstics(const Eigen::Matrix3f& originalCameraIntrinstics, Level level);            
    };
} // namespace kinect_fusion