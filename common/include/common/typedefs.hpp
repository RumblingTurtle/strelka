#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// 3x1 Vector
template <typename T> using Vec3 = typename Eigen::Matrix<T, 3, 1>;

// 4x1 Vector
template <typename T> using Vec4 = typename Eigen::Matrix<T, 4, 1>;

// 12x1 Vector
template <typename T> using Vec12 = Eigen::Matrix<T, 12, 1>;

// 3x3 Matrix
template <typename T> using Mat3 = typename Eigen::Matrix<T, 3, 3>;

// 4x1 Vector
template <typename T> using Quat = typename Eigen::Matrix<T, 4, 1>;

// 4x3 Matrix
template <typename T> using Mat43 = typename Eigen::Matrix<T, 4, 3>;

// Dynamically sized matrix
template <typename T>
using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template <typename T> using DVec = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;
#endif // TYPEDEFS_H