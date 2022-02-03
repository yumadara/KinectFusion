#include <surface_measurement_utils_cuda.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/core/cuda.hpp>

namespace kinect_fusion {

// __global__
// void fillVertexMapKernel(const Map2DfCuda& depths, Map2DVector3fCuda& vertexMap, float fovX, float fovY, float cX, float cY) {
//     // Get depth intrinsics.
//     const int row = blockIdx.x * blockDim.x + threadIdx.x;
//     const int col = blockIdx.y * blockDim.y + threadIdx.y;

//     std::size_t idx = depths.getIndex(row, col); // linearized index
//     float depth = depths.get(idx);
//     if (depth == MINF) {
//         vertexMap.set(idx, Vector3f(MINF, MINF, MINF));
//     }
//     else {
//         // Back-projection to camera space.
//         vertexMap.set(idx, Vector3f((col - cX) / fovX * depth, (row - cY) / fovY * depth, depth));
//     }
// }

// void fillVertexMap(const Map2DfCuda& depths, const Eigen::Matrix3f& depthIntrinsics, Map2DVector3fCuda& vertexMap)
// {
//     float fovX = depthIntrinsics(0, 0);
//     float fovY = depthIntrinsics(1, 1);
//     float cX = depthIntrinsics(0, 2);
//     float cY = depthIntrinsics(1, 2);

//     dim3 threads(32, 32);
//     dim3 blocks((depths.getNumberOfRows() + threads.x - 1) / threads.x, 
//                     (depths.getNumberOfColumns() + threads.y - 1) / threads.y);
//     fillVertexMapKernel<<<blocks, threads>>>(depths, vertexMap, fovX, fovY, cX, cY);
//     cudaThreadSynchronize();
// }

// __global__
// void fillNormalMapKernel(const Map2DVector3fCuda& vertexMap, Map2DVector3fCuda& normalMap) {

//     const int row = blockIdx.x * blockDim.x + threadIdx.x;
//     const int col = blockIdx.y * blockDim.y + threadIdx.y;

//     std::size_t idx = vertexMap.getIndex(row, col); // linearized index

//     auto firstVector{vertexMap.get(idx + vertexMap.getWidth()) - vertexMap.get(idx)};
//     auto secondVector{vertexMap.get(idx + 1) - vertexMap.get(idx)};

//     if (!firstVector.allFinite() || !secondVector.allFinite()) {
//         normalMap.get(idx) = Vector3f(MINF, MINF, MINF);
//     } else {
//         normalMap.get(idx) = firstVector.cross(secondVector);
//         normalMap.get(idx).normalize();
//     }       
// }

// void fillNormalMap(const Map2DVector3fCuda& vertexMap, Map2DVector3fCuda& normalMap) {
//     dim3 threads(32, 32);
//     dim3 blocks((vertexMap.getNumberOfRows() + threads.x - 1) / threads.x, 
//                     (vertexMap.getNumberOfColumns() + threads.y - 1) / threads.y);
//     fillNormalMapKernel<<<blocks, threads>>>(vertexMap, normalMap);
//     cudaThreadSynchronize();
// }

__global__
void subsampleKernel(const Map2DfCuda& previousDepthMap, Map2DfCuda& nextDepthMap)
{
    const int row = blockIdx.x * blockDim.x + threadIdx.x;
    const int col = blockIdx.y * blockDim.y + threadIdx.y;

    const float centerPixel = previousDepthMap.get(row, col);
    float& newPixel{nextDepthMap.get(row / 2, col / 2)};

    if (centerPixel == MINF) {
        newPixel = MINF;
        return;
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

void subsample(const Map2DfCuda& previousDepthMap, Map2DfCuda& nextDepthMap) {
    dim3 threads(32, 32);
    dim3 blocks((previousDepthMap.getNumberOfRows() + threads.x - 1) / threads.x, 
                    (previousDepthMap.getNumberOfColumns() + threads.y - 1) / threads.y);

    subsampleKernel<<<blocks, threads>>>(previousDepthMap, nextDepthMap);
    cudaThreadSynchronize();
}

void applyBiliteralFilter(Map2DfCuda& unfilteredMap, Map2DfCuda& filteredMap) {
    const float* a = thrust::raw_pointer_cast(unfilteredMap.data());
    const cv::cudev::GpuMat_<float> cvUnfilteredMap{unfilteredMap.getNumberOfColumns(), 
                    unfilteredMap.getNumberOfRows(), a};
    
    a = thrust::raw_pointer_cast(unfilteredMap.data());
    cv::cudev::GpuMat_<float> cvFilteredMap{filteredMap.getNumberOfColumns(), filteredMap.getNumberOfRows(), 
                a};

    constexpr float BIG_NEGATIVE_NUMBER = -10000.0; // -infinity leads to nan, so use

    // for (std::size_t i = 0; i < unfilteredMap.size(); i++) {
    //     if (unfilteredMap[i] == MINF) {
    //         unfilteredMap[i] = BIG_NEGATIVE_NUMBER;
    //     }
    // }
    cv::cuda::Stream stream;
    cv::bilateralFilter(cvUnfilteredMap, cvFilteredMap, FILTER_SIZE, SIGMA_DEPTH, SIGMA_SPACE, stream);
    // for (std::size_t i = 0; i < unfilteredMap.size(); i++) {
    //     if (unfilteredMap[i] == BIG_NEGATIVE_NUMBER) {
    //         unfilteredMap[i] = MINF;
    //         filteredMap[i] = MINF;
    //     }
    // }
}
} // namespace kinect_fusion