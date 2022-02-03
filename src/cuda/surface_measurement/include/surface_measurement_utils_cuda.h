#pragma once

#include <Eigen.h>

#include <type_definitions_cuda.h>
#include <surface_measurement_utils.h>

namespace kinect_fusion {

/**
 * @brief Fill vertex map given depths values and camera instinsics using
 * VertexMap(row, column) = DepthMap(row, column) * K^(-1) * (row, column, 1);
 * 
 * @param[in] depths Depth map
 * @param[in] cameraIntrinsics Camera intrinsics, or K matrix
 * @param[out] vertexMap Vertex map that should be filled
 */
void fillVertexMap(const Map2DfCuda& depths, const Eigen::Matrix3f& cameraIntrinsics, Map2DVector3fCuda& vertexMap);

/**
 * @brief Fill normal map given vertex map (V) using
 * NormalMap(row, col) = (V(row + 1, col) − V(row, col)) × (V(row, col + 1) - V(row, col))
 * NormalMap(row, col) = NormalMap(row, col) / || NormalMap(row, col) ||
 * 
 * @param[in] vertexMap Vertex map
 * @param[out] normalMap Normal Map, which should be filled
 */
void fillNormalMap(const Map2DVector3fCuda& vertexMap, Map2DVector3fCuda& normalMap);

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
void subsample(const Map2DfCuda& previousDepthMap, Map2DfCuda& nextDepthMap);

/**
 * @brief Apply biliteral filter.
 * 
 * @param[in] unfilteredMap Input unfiltered map
 * @param[out] filteredMap Output filtered map that should be filled
 */
void applyBiliteralFilter(Map2DfCuda& unfilteredMap, Map2DfCuda& filteredMap);

} // namespace surface_measurement