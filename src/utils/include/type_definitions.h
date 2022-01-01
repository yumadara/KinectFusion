#pragma once

#include <Eigen.h>

namespace kinect_fusion {

    template <typename T>
    class Map2D {

        public:
            Map2D() {};

            Map2D(std::size_t height, std::size_t width) : dataVector(height * width), m_height(height), m_width(width) 
            {}

            inline T& get(std::size_t index) {
                return dataVector[index];
            }

            inline const T& get(std::size_t index) const {
                return dataVector[index];
            }

            inline T& get(std::size_t row, std::size_t column) {
                return dataVector[column + row * m_width];
            }

            inline const T& get(std::size_t row, std::size_t column) const {
                return dataVector[column + row * m_width];
            }

            inline T& operator[](std::size_t index) {
                return dataVector[index];
            }

            inline const T& operator[](std::size_t index) const {
                return dataVector[index];
            }

            inline std::size_t getIndex(std::size_t row, std::size_t column) const {
                return column + row * m_width;
            }

            inline std::vector<T> getDataVector() {
                return dataVector;
            }

            inline std::size_t height() const {
                return m_height;
            }

            inline std::size_t width() const {
                return m_width;
            }

            inline T* data() {
                return dataVector.data();
            }

            inline std::size_t size() {
                return dataVector.size();
            }

        private: 
            std::vector<T> dataVector;

            std::size_t m_height;
            std::size_t m_width;
    };

    typedef Map2D<float> Map2Df;

    typedef Map2D<Eigen::Vector3f> Map2DVector3f;
}