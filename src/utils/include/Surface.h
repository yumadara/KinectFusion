#pragma once

#include <cassert>

#include "Eigen.h"

#include <type_definitions.h>

namespace kinect_fusion {

class Surface {
    public:
        Surface(std::size_t height, std::size_t width) :
            vertexMap(height, width, Vector3f::Zero()),
            normalMap(height, width, Vector3f::Zero())
            {}

        Surface() {}

        inline void setVertex(std::size_t row, std::size_t column, const Vector3f& value) {
            vertexMap.set(row, column, value);
        }
        inline void setNormal(std::size_t row, std::size_t column, const Vector3f& value) {
            vertexMap.set(row, column, value);
        }

        inline Vector3f& getVertex(std::size_t row, std::size_t column) {
            return vertexMap.get(row, column);
        }
        inline const Vector3f& getVertex(std::size_t row, std::size_t column) const {
            return vertexMap.get(row, column);
        }

        inline Vector3f& getNormal(std::size_t row, std::size_t column) {
            return vertexMap.get(row, column);
        }
        inline const Vector3f& getNormal(std::size_t row, std::size_t column) const {
            return vertexMap.get(row, column);
        }

        inline std::size_t getHeight(){
            return vertexMap.getHeight();
        }

        inline std::size_t getWidth(){
            return vertexMap.getWidth();
        }

        inline Map2DVector3f& getVertexMap() {
            return vertexMap;
        }

        inline Map2DVector3f& getNormalMap() {
            return normalMap;
        }

    private:
        Map2DVector3f vertexMap;
        Map2DVector3f normalMap;
    };
} // namespace kinect_fusion