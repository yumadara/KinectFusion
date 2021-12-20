#pragma once

#include <Eigen.h>

namespace kinect_fusion {
    typedef Eigen::Matrix<float, Dynamic, Dynamic, Eigen::RowMajor> MatrixXf;
    
    constexpr std::size_t NUMBER_OF_LEVELS = 3;

    class FrameData {
        public:
            FrameData(float* depths, std::size_t width, std::size_t height, const Eigen::Matrix3f& cameraIntrinstics);

        private:
            MatrixXf m_rowDepthMap;
            MatrixXf m_depthMaps[NUMBER_OF_LEVELS];
            MatrixXf m_normalMaps[NUMBER_OF_LEVELS];
            
            Eigen::Matrix3f m_cameraIntrinstics[NUMBER_OF_LEVELS];
    };
} // namespace kinect_fusion