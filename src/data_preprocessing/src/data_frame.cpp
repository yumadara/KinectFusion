#include <data_frame.h>

namespace kinect_fusion {
    FrameData::FrameData(const MatrixXfPointer& depths, std::size_t width, std::size_t height, const Eigen::Matrix3f& cameraIntrinstics) :
        m_rowDepthMap(Convert(depths))
        {
            for (std::size_t i = 0; i < NUMBER_OF_LEVELS; ++i) {
                Level level = static_cast<Level>(i + 1);
                m_cameraIntrinstics[level - 1] = getLevelCameraIntrinstics(cameraIntrinstics, level);

            }
        }

    Eigen::Matrix3f FrameData::getLevelCameraIntrinstics(const Eigen::Matrix3f& originalCameraIntrinstics, Level level)
    {
        Eigen::Matrix3f levelCameraIntrinstics{originalCameraIntrinstics};

        if (level == Level::First) {
            return levelCameraIntrinstics;
        }

        float scale = std::pow(0.5f, static_cast<std::size_t>(level));

        levelCameraIntrinstics(0, 0) *= scale; // focal_x
        levelCameraIntrinstics(1, 1) *= scale; // focal y
        levelCameraIntrinstics(0, 1) *= scale; // skew coefficient

        // Pixel coordinates origin relative to the borders.
        // (0, 0) pixel is actually in the center of the first pixel, so there is 0.5 pixel to the border.
        constexpr float pixelCoordinatesOrigin = 0.5;

        // Get principal point
        float& principalX = levelCameraIntrinstics(0, 2);
        float& principalY = levelCameraIntrinstics(1, 2);

        // Since pixels size changed, pixel coordinate origin (i.e. pixel 0, 0) has also also yet another position
        // However, borders are still at the same place. So, we have to scale the distance between pixels and border, 
        // instead the distance between pixels and the first pixel
        principalX = (principalX + pixelCoordinatesOrigin) * scale - pixelCoordinatesOrigin;
        principalY = (principalY + pixelCoordinatesOrigin) * scale - pixelCoordinatesOrigin;

        return levelCameraIntrinstics;
    }
    
    cv::Mat_<float> Convert(const MatrixXfPointer& matrix) {
        cv::Mat_<float> cvMatrix(matrix->rows(), matrix->cols());
        memcpy(cvMatrix.data, matrix->data(), matrix->rows() * matrix->cols() * sizeof(float));
        return cvMatrix;
    }
} // namespace kinect_fusion