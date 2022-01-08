#pragma once

#include <Eigen.h>

namespace kinect_fusion {

/**
 * @brief 2D map wrapper for row major array with columns and rows. 
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
         * 
         * @return std::size_t Number of rows, or height.
         */
        inline std::size_t getHeight() const {
            return m_height;
        }

        /**
         * @brief Get number of columns, or width of 2D map.
         * 
         * @return std::size_t Number of columns, or width.
         */
        inline std::size_t getWidth() const {
            return m_width;
        }

        /**
         * @brief Get underlying array as pointer.
         * 
         * @return T* Underlying array as pointer.
         */
        inline T* data() {
            return dataVector.data();
        }

        /**
         * @brief Get size of the array.
         * 
         * @return std::size_t Size
         */
        inline std::size_t size() {
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


/**
 * @brief Levels.
 */
enum class Level : std::size_t {
    First = 1U,
    Second = 2U,
    Third = 3U
};

/**
 * @brief Number of levels.
 */
constexpr std::size_t NUMBER_OF_LEVELS = 3;

/**
 * @brief All levels (1, 2, 3).
 */
constexpr Level LEVELS[NUMBER_OF_LEVELS] {
    Level::First, Level::Second, Level::Third
};

} // namespace kinect_fusion