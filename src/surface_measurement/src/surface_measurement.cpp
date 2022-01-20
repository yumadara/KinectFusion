#include <surface_measurement_utils.h>

#include <opencv2/imgproc.hpp>

namespace kinect_fusion {

/**
 * @brief Camera intrinstics focal scale.
 */
constexpr float FOCAL_SCALE = 525.0f; // pixels/m

/**
 * @brief Filter sigma in the depth space (sigma_r). A larger value of the parameter means that farther 
 * colors within the pixel neighborhood (see SIGMA_SPACE) will be mixed together, resulting in larger areas of semi-equal color.
 * 
 * @note For simplicity, you can set the 2 sigma values to be the same. If they are small (< 10), 
 * the filter will not have much effect, whereas if they are large (> 150), 
 * they will have a very strong effect, making the image look "cartoonish".
 */
constexpr float SIGMA_DEPTH = 0.033f; // 3.3 cm

/**
 * @brief Sigma for distances (sigma_s) for bilateral filter. A larger value of the parameter means that 
 * farther pixels will influence each other as long as their depths are close enough (see SIGMA_DEPTH).
 */
constexpr float SIGMA_SPACE = 0.1f * FOCAL_SCALE; // 10 cm, 0.1 * 525.0 = 52.5 pixels

/**
 * @brief Diameter of each pixel neighborhood that is used during filtering, or d. 
 * If it is non-positive, it is computed from sigmaSpace.
 * 
 * @note Large filters (filter size > 5) are very slow, so it is recommended to use filter size = 5 for real-time applications, 
 * and perhaps filter size = 9 for offline applications that need heavy noise filtering.
 */
constexpr int FILTER_SIZE = 5;

/**
 * @brief Max distance, or 3 sigmas. For subsampling, when averaging the block of pixels,
 * if value of neighbor pixel is MAX_DISTANCE greater/smaller than the value of center pixel,
 * it is not used for averaging. 
 */
constexpr float MAX_DISTANCE = SIGMA_DEPTH * 3.0f;

void fillVertexMap(const Map2Df& depths, const Eigen::Matrix3f& depthIntrinsics, Map2DVector3f& vertexMap) {
    // Get depth intrinsics.
    float fovX = depthIntrinsics(0, 0);
    float fovY = depthIntrinsics(1, 1);
    float cX = depthIntrinsics(0, 2);
    float cY = depthIntrinsics(1, 2);

            // For every pixel row.
    #pragma omp parallel for
            for (int row = 0; row < depths.getHeight(); ++row) {
                // For every pixel in a row.
                for (int col = 0; col < depths.getWidth(); ++col) {
                    std::size_t idx = depths.getIndex(row, col); // linearized index
                    float depth = depths.get(idx);
                    if (depth == MINF) {
                        vertexMap.set(idx, Vector3f(MINF, MINF, MINF));
                    }
                    else {
                        // Back-projection to camera space.
                        vertexMap.set(idx, Vector3f((col - cX) / fovX * depth, (row - cY) / fovY * depth, depth));
                    }
                }
            }
        }

void fillNormalMap(const Map2DVector3f& vertexMap, Map2DVector3f& normalMap) {

#pragma omp parallel for
    for (std::size_t row = 1; row < vertexMap.getHeight() - 1; ++row) {
        for (std::size_t col = 1; col < vertexMap.getWidth() - 1; ++col) {
            std::size_t idx = vertexMap.getIndex(row, col); // linearized index

            auto firstVector{vertexMap.get(idx + vertexMap.getWidth()) - vertexMap.get(idx)};
            auto secondVector{vertexMap.get(idx + 1) - vertexMap.get(idx)};

            if (!firstVector.allFinite() || !secondVector.allFinite()) {
                normalMap.get(idx) = Vector3f(MINF, MINF, MINF);
            } else {
                normalMap.get(idx) = firstVector.cross(secondVector);
                normalMap.get(idx).normalize();
            }       
        }
    }
}

void subsample(const Map2Df& previousDepthMap, Map2Df& nextDepthMap)
{
    for (int row = 0; row < previousDepthMap.getHeight() - 1; row += 2) {
        for (int col = 0; col < previousDepthMap.getWidth() - 1; col += 2) {
            const float centerPixel = previousDepthMap.get(row, col);
            float& newPixel = nextDepthMap.get(row / 2, col / 2);

            if (centerPixel == MINF) {
                newPixel = MINF;
                continue;
            }

            int topNeighbourRow = row - 1;
            int leftNeighbourCol = col - 1;
            int numberOfPixels = 9;

            if (col == 0) {
                leftNeighbourCol = 0;
                numberOfPixels = 6;
            }
            if (row == 0){
                topNeighbourRow = 0;
                numberOfPixels = 6;
            }
            if (col == 0 && row == 0)
            {
                numberOfPixels = 4;
            }

            newPixel = 0;
            for (int neighbourRow = topNeighbourRow; neighbourRow <= row + 1; neighbourRow++) {
                for (int neighbourCol = leftNeighbourCol; neighbourCol <= col + 1; neighbourCol++) {
                    float neighbourValue = previousDepthMap.get(neighbourRow, neighbourCol);
                    if (std::abs(neighbourValue - centerPixel) > MAX_DISTANCE) {
                        numberOfPixels--;
                    } else {
                        newPixel += neighbourValue;
                    }
                }                 
            }
            newPixel /= numberOfPixels;
        }
    }
}

void applyBiliteralFilter(Map2Df& unfilteredMap, Map2Df& filteredMap) {
    const cv::Mat cvUnfilteredMap(unfilteredMap.getNumberOfColumns(), unfilteredMap.getNumberOfRows(), CV_32F, 
                reinterpret_cast<void*>(unfilteredMap.data()));
    cv::Mat cvFilteredMap(filteredMap.getNumberOfColumns(), filteredMap.getNumberOfRows(), CV_32F, 
                reinterpret_cast<void*>(filteredMap.data()));

    constexpr float BIG_NEGATIVE_NUMBER = -10000.0; // -infinity leads to nan, so use

    for (std::size_t i = 0; i < unfilteredMap.size(); i++) {
        if (unfilteredMap[i] == MINF) {
            unfilteredMap[i] = BIG_NEGATIVE_NUMBER;
        }
    }
    cv::bilateralFilter(cvUnfilteredMap, cvFilteredMap, FILTER_SIZE, SIGMA_DEPTH, SIGMA_SPACE);
    for (std::size_t i = 0; i < unfilteredMap.size(); i++) {
        if (unfilteredMap[i] == BIG_NEGATIVE_NUMBER) {
            unfilteredMap[i] = MINF;
            filteredMap[i] = MINF;
        }
    }
}

Eigen::Matrix3f computeLevelCameraIntrinstics(const Eigen::Matrix3f& originalCameraIntrinstics, Level level) 
    {
        Eigen::Matrix3f levelCameraIntrinstics{originalCameraIntrinstics};

        if (level == 0U) {
            return levelCameraIntrinstics;
        }

        float scale = std::pow(0.5f, level);

        levelCameraIntrinstics(0, 0) *= scale; // focal_x
        levelCameraIntrinstics(1, 1) *= scale; // focal y
        levelCameraIntrinstics(0, 1) *= scale; // skew coefficient

        levelCameraIntrinstics(0, 2) *= scale; // principal point x
        levelCameraIntrinstics(1, 2) *= scale; // principal point y

        return levelCameraIntrinstics;
    }
} // namespace kinect_fusion