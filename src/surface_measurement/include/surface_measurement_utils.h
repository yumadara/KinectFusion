#pragma once

#include <Eigen.h>

#include <type_definitions.h>

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

namespace kinect_fusion {

/**
 * @brief Fill vertex map given depths values and camera instinsics using
 * VertexMap(row, column) = DepthMap(row, column) * K^(-1) * (row, column, 1);
 * 
 * @param[in] depths Depth map
 * @param[in] cameraIntrinsics Camera intrinsics, or K matrix
 * @param[out] vertexMap Vertex map that should be filled
 */
void fillVertexMap(const Map2Df& depths, const Eigen::Matrix3f& cameraIntrinsics, Map2DVector3f& vertexMap);

/**
 * @brief Fill normal map given vertex map (V) using
 * NormalMap(row, col) = (V(row + 1, col) − V(row, col)) × (V(row, col + 1) - V(row, col))
 * NormalMap(row, col) = NormalMap(row, col) / || NormalMap(row, col) ||
 * 
 * @param[in] vertexMap Vertex map
 * @param[out] normalMap Normal Map, which should be filled
 */
void fillNormalMap(const Map2DVector3f& vertexMap, Map2DVector3f& normalMap);

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
void subsample(const Map2Df& previousDepthMap, Map2Df& nextDepthMap);

/**
 * @brief Apply bilateral filter.
 * 
 * @param[in] unfilteredMap Input unfiltered map
 * @param[out] filteredMap Output filtered map that should be filled
 */
void applyBilateralFilter(Map2Df& unfilteredMap, Map2Df& filteredMap);

/**
 * @brief Get the Camera Intrinstics for specific level (see surface measurement section in the paper for level definitions)
 * 
 * @param originalCameraIntrinstics Camera Intrinstics for first level, or real camera intrinstics
 * @param level Level for which camera intrinstics should be computed
 * @return Eigen::Matrix3f Camera intrinstics for this level.
 */
Eigen::Matrix3f computeLevelCameraIntrinstics(const Eigen::Matrix3f& originalCameraIntrinstics, Level level);

} // namespace surface_measurement