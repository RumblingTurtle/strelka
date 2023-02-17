#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// Rotation Matrix
template <typename T> using RotMat = typename Eigen::Matrix<T, 3, 3>;

// 2x1 Vector
template <typename T> using Vec2 = typename Eigen::Matrix<T, 2, 1>;

// 3x1 Vector
template <typename T> using Vec3 = typename Eigen::Matrix<T, 3, 1>;

// 4x1 Vector
template <typename T> using Vec4 = typename Eigen::Matrix<T, 4, 1>;

// 4x1 Vector
template <typename T> using Vec43 = typename Eigen::Matrix<T, 4, 3>;

// 6x1 Vector
template <typename T> using Vec6 = Eigen::Matrix<T, 6, 1>;

// 10x1 Vector
template <typename T> using Vec10 = Eigen::Matrix<T, 10, 1>;

// 12x1 Vector
template <typename T> using Vec12 = Eigen::Matrix<T, 12, 1>;

// 12x1 Vector
template <typename T> using Vec9 = Eigen::Matrix<T, 9, 1>;

// 18x1 Vector
template <typename T> using Vec18 = Eigen::Matrix<T, 18, 1>;

// 28x1 vector
template <typename T> using Vec28 = Eigen::Matrix<T, 28, 1>;

// 3x3 Matrix
template <typename T> using Mat3 = typename Eigen::Matrix<T, 3, 3>;

// 4x1 Vector
template <typename T> using Quat = typename Eigen::Matrix<T, 4, 1>;

// 6x6 Matrix
template <typename T> using Mat6 = typename Eigen::Matrix<T, 6, 6>;

// 12x12 Matrix
template <typename T> using Mat12 = typename Eigen::Matrix<T, 12, 12>;

// 18x18 Matrix
template <typename T> using Mat18 = Eigen::Matrix<T, 18, 18>;

// 28x28 Matrix
template <typename T> using Mat28 = Eigen::Matrix<T, 28, 28>;

// 3x4 Matrix
template <typename T> using Mat34 = Eigen::Matrix<T, 3, 4>;

// 3x4 Matrix
template <typename T> using Mat23 = Eigen::Matrix<T, 2, 3>;

// 4x3 Matrix
template <typename T> using Mat43 = typename Eigen::Matrix<T, 4, 3>;

// Dynamically sized matrix
template <typename T>
using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template <typename T> using DVec = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;
#endif // TYPEDEFS_H