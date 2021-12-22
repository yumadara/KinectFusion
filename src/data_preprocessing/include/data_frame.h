#pragma once

#include <Eigen.h>

#include <opencv2/core/mat.hpp>

namespace kinect_fusion {
    typedef Eigen::Matrix<float, Dynamic, Dynamic, Eigen::RowMajor> MatrixXf;
    typedef std::shared_ptr<MatrixXf> MatrixXfPointer;
    
    constexpr std::size_t NUMBER_OF_LEVELS = 3;

    constexpr float SIGMA{0.1};

    /**
     * @brief Levels.
     */
    enum Level : std::size_t {
        First = 1U,
        Second = 2U,
        Third = 3U
    };

    class FrameData {
        public:
            /**
             * @brief Construct a new Frame Data object
             * 
             * @param depths depth map from sensor
             * @param width of depth map
             * @param height of depth map
             * @param cameraIntrinstics camera instinstics
             */
            FrameData(const MatrixXfPointer& depths, std::size_t width, std::size_t height, const Eigen::Matrix3f& cameraIntrinstics);

        private:
            /**
             * @brief Row depth map from sensor
             */
            cv::Mat_<float> m_rowDepthMap;

            /**
             * @brief Filtered depth maps for different levels
             */
            cv::Mat_<float> m_depthMaps[NUMBER_OF_LEVELS];

            /**
             * @brief Normal maps based on filtered depth maps for different levels
             */
            cv::Mat_<float> m_normalMaps[NUMBER_OF_LEVELS];
            
            /**
             * @brief Camera instrinstics for different levels.
             */
            Eigen::Matrix3f m_cameraIntrinstics[NUMBER_OF_LEVELS];

            /**
             * @brief Convert Eigen Matrix to cv::Mat structure.
             * 
             * @param matrix Pointer to the eigen matrix
             * @return cv::Mat_<float> 
             */
            static cv::Mat_<float> Convert(const MatrixXfPointer& matrix);

            /**
             * @brief Get the Camera Intrinstics for specific level;
             * 
             * @param originalCameraIntrinstics Camera Intrinstics for level 0, or real camera intrinstics
             * @param level for which camera intrinstics should be computed
             * @return Eigen::Matrix3f camera intrinstics for this elvel.
             */
            static Eigen::Matrix3f getLevelCameraIntrinstics(const Eigen::Matrix3f& originalCameraIntrinstics, Level level);

            /**
             * @brief Get the normal map given the depth map.
             * 
             * @param depths Depth map for which normal map should be computed
             * @return cv::Mat_<float> Normal map
             */
            static cv::Mat_<float> GetNormalMap(const cv::Mat_<float>& depths);

            /**
             * @brief Subsample the depths for next level to half resolution, using averages and disregarding values 
             * that are more than 3 sigmas greater/smaller than the central pixel.
             * 
             * @param depths Depth map
             * @return cv::Mat_<float> Depth map for next level
             */
            static cv::Mat_<float> Subsample(const cv::Mat_<float>& depths);




            
    };
} // namespace kinect_fusion