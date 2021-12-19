#include <data_frame.h>

namespace kinect_fusion {
    FrameData::FrameData(float* depths, std::size_t width, std::size_t height, const Eigen::Matrix3f& cameraIntrinstics) :
        m_rowDepthMap(width, height)
        {
            std::size_t numberOfPixels = width * height;
            memcpy(m_rowDepthMap.data(), depths, numberOfPixels);

        }
} // namespace kinect_fusion