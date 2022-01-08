#pragma once

#include <cassert>

#include "Eigen.h"

#include <type_definitions.h>

namespace kinect_fusion {

/**
 * @brief Wrapper around 2D vertex and normal maps.
 * 
 * @note 2D Vertex map maps pixel as (row, column) to the coordinates in camera
 * coordinate system, while 2D normal map maps pixels to the normal vectors.
 * 
 * @note If there is no surface at some pixel, or normal could not be computed, 
 * the values will be -infinity
 */
class Surface {
    public:
        /**
         * @brief Main constructor. Creates vertex and normal maps and initializes it with zero vectors.
         * 
         * @param height Height, or number of pixel rows in the maps 
         * @param width Width, or number of pixel columns in the maps
         */
        Surface(std::size_t height, std::size_t width) :
            vertexMap(height, width, Vector3f::Zero()),
            normalMap(height, width, Vector3f::Zero())
            {}

        /**
         * @brief Construct an empty Surface object. 
         */
        Surface() {}

        /**
         * @brief Set vertex (i.e. coordinate in camera coordinate system) at (row, column) pixel.
         * 
         * @param row Row of the pixel
         * @param column Column of the pixel
         * @param value Value which will be set
         */
        inline void setVertex(std::size_t row, std::size_t column, const Vector3f& value) {
            vertexMap.set(row, column, value);
        }

        /**
         * @brief Set normal vector at (row, column) pixel.
         * 
         * @param row Row of the pixel
         * @param column Column of the pixel
         * @param value Value which will be set
         */
        inline void setNormal(std::size_t row, std::size_t column, const Vector3f& value) {
            vertexMap.set(row, column, value);
        }

        /**
         * @brief Get vertex (i.e. coordinate in camera coordinate system) at (row, column) pixel.
         * 
         * @param row Row of the pixel
         * @param column Column of the pixel
         * @return Vertex at (row, column) pixel
         */
        inline Vector3f& getVertex(std::size_t row, std::size_t column) {
            return vertexMap.get(row, column);
        }
        inline const Vector3f& getVertex(std::size_t row, std::size_t column) const {
            return vertexMap.get(row, column);
        }

        /**
         * @brief Get normal vector at (row, column) pixel.
         * 
         * @param row Row of the pixel
         * @param column Column of the pixel
         * @return Normal vector at (row, column) pixel
         */
        inline Vector3f& getNormal(std::size_t row, std::size_t column) {
            return vertexMap.get(row, column);
        }
        inline const Vector3f& getNormal(std::size_t row, std::size_t column) const {
            return vertexMap.get(row, column);
        }

        /**
         * @brief Get height or number of pixel rows.
         */
        inline std::size_t getHeight(){
            return vertexMap.getHeight();
        }

        /**
         * @brief Get width or number of pixel columns.
         */
        inline std::size_t getWidth(){
            return vertexMap.getWidth();
        }

        /**
         * @brief Get the vertex map, i.e. map from pixel as (row, column) to the coordinates in camera
         * coordinate system
         * 
         * @return Map2DVector3f& Vertex map
         */
        inline Map2DVector3f& getVertexMap() {
            return vertexMap;
        }

        /**
         * @brief Get the normal map, i.e. maps from pixels as (row, column) to the normal vectors.
         * 
         * @return Map2DVector3f& Vertex map
         */
        inline Map2DVector3f& getNormalMap() {
            return normalMap;
        }

    private:
        /// See Surface description.
        Map2DVector3f vertexMap;
        Map2DVector3f normalMap;
    };
} // namespace kinect_fusion