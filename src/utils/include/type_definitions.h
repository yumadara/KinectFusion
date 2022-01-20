#pragma once

#include <Eigen.h>

namespace kinect_fusion {

/**
 * @brief 2D map wrapper for row major array with rows and columns. 
 * 
 * @note Is mainly used of map pixels as (row, column) to values.
 * 
 * @tparam T Type of values
 */
template <typename T>
class Map2D {

    public:
        /**
         * @brief Constructs empty 2D map. It is used to create placeholders.
         */
        Map2D() {};

        /**
         * @brief Construct a new Map 2D object
         * 
         * @param height Number of rows
         * @param width Number of columns
         * @param initialValue Initial value for the array
         */
        Map2D(std::size_t height, std::size_t width, const T& initialValue = T()) : 
            dataVector(height * width, initialValue), 
            m_height(height), 
            m_width(width) 
        {
        }

        /**
         * @brief Get value at specific index from underlying array.
         * 
         * @param index Index, where value is in underlying array.
         * @return T& value
         */
        inline T& get(std::size_t index) {
            return dataVector[index];
        }
        inline const T& get(std::size_t index) const {
            return dataVector[index];
        }

        /**
         * @brief Get value at specific row and column
         * 
         * @param index Index a which the value is located
         * @return const T& value
         */
        inline T& get(std::size_t row, std::size_t column) {
            return dataVector[column + row * m_width];
        }
        inline const T& get(std::size_t row, std::size_t column) const {
            return dataVector[column + row * m_width];
        }

        /**
         * @brief Set value at specific row and column
         * 
         * @param index Index a which the value should be set
         */
        inline void set(std::size_t row, std::size_t column, const T& value) {
            get(row, column) = value;
        }

        inline void set(std::size_t index, const T& value) {
            get(index) = value;
        }

        /**
         * @brief Subscription operator. It does the same as get method.
         * 
         * @param index Index a which the value is located
         * @return T& value
         */
        inline T& operator[](std::size_t index) {
            return dataVector[index];
        }
        inline const T& operator[](std::size_t index) const {
            return dataVector[index];
        }

        /**
         * @brief Get index of the value located at given row and column according to the equation
         * for row major arrays: index = column + row * width.
         * 
         * @param row Row index
         * @param column Column index
         * @return std::size_t Index in the row major array
         */
        inline std::size_t getIndex(std::size_t row, std::size_t column) const {
            return column + row * m_width;
        }

        /**
         * @brief Get underlying array as std::vector.
         * 
         * @return std::vector<T> Underlying array as std::vector.
         */
        inline std::vector<T> getDataVector() {
            return dataVector;
        }

        /**
         * @brief Get number of rows, or height of 2D map.
         */
        inline std::size_t getHeight() const {
            return m_height;
        }

        /**
         * @brief Get number of rows, or height.
         */
        inline std::size_t getNumberOfRows() const {
            return getHeight();
        }

        /**
         * @brief Get number of columns, or width of 2D map.
         */
        inline std::size_t getWidth() const {
            return m_width;
        }

        /**
         * @brief Get number of rows, or height.
         */
        inline std::size_t getNumberOfColumns() const {
            return getWidth();
        }

        /**
         * @brief Get underlying array as pointer.
         */
        inline T* data() {
            return dataVector.data();
        }
        inline const T* data() const {
            return dataVector.data();
        }

        /**
         * @brief Get size of the array.
         */
        inline std::size_t size() const {
            return dataVector.size();
        }

    private: 
        std::vector<T> dataVector;
        std::size_t m_height;
        std::size_t m_width;
};

/// 2D Map for float numbers
typedef Map2D<float> Map2Df;

/// 2D Map for Eigen float vectors
typedef Map2D<Eigen::Vector3f> Map2DVector3f;

/// Type of level
typedef std::size_t Level;

/**
 * @brief Number of levels;
 */
constexpr std::size_t NUMBER_OF_LEVELS = 3;

/**
 * @brief Levels by name
 */
constexpr Level FIRST_LEVEL{0U}, SECOND_LEVEL{1U}, THIRD_LEVEL{2U};

/**
 * @brief Arrays containing 3 levels;
 */
constexpr Level LEVELS[NUMBER_OF_LEVELS] {
    FIRST_LEVEL, SECOND_LEVEL, THIRD_LEVEL
};

} // namespace kinect_fusion