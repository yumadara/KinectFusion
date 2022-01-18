#include <surface_measurement_utils.h>

namespace kinect_fusion {

/**
 * @brief Max distance, or 3 sigmas. For subsampling, when averaging the block of pixels,
 * if value of neighbor pixel is MAX_DISTANCE greater/smaller than the value of center pixel,
 * it is not used for averaging. 
 */
constexpr float MAX_DISTANCE = 0.1f;

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
                        //vertexMap.get(idx) = Vector3f(MINF, MINF, MINF);
                        vertexMap.set(idx, Vector3f(MINF, MINF, MINF));
                    }
                    else {
                        // Back-projection to camera space.
                        //vertexMap.get(idx) = Vector3f((col - cX) / fovX * depth, (row - cY) / fovY * depth, depth);
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

            float centerPixel = previousDepthMap.get(row, col);
            float& newPixel = nextDepthMap.get(int(row / 2), int(col / 2));
            newPixel = 0;

            for (int neighbourRow = topNeighbourRow; neighbourRow <= row + 1; neighbourRow++) {
                for (int neighbourCol = leftNeighbourCol; neighbourCol <= col + 1; neighbourCol++) {
                    float neighbourValue = previousDepthMap.get(neighbourRow, neighbourCol);
                    if (std::abs(neighbourValue - centerPixel) > MAX_DISTANCE) {
                        numberOfPixels--;
                    } else {
                        newPixel += neighbourValue;
                    }
                    newPixel /= numberOfPixels;
                }                 
            }
        }
    }

}

Eigen::Matrix3f computeLevelCameraIntrinstics(const Eigen::Matrix3f& originalCameraIntrinstics, Level level) 
    {
        Eigen::Matrix3f levelCameraIntrinstics{originalCameraIntrinstics};

        if (level == Level::First) {
            return levelCameraIntrinstics;
        }

        float scale = std::pow(0.5f, static_cast<std::size_t>(level));

        levelCameraIntrinstics(0, 0) *= scale; // focal_x
        levelCameraIntrinstics(1, 1) *= scale; // focal y
        levelCameraIntrinstics(0, 1) *= scale; // skew coefficient
        // Since first pixel is as it was at (0, 0), we do not need to change principal point!

        return levelCameraIntrinstics;
    }
} // namespace kinect_fusion