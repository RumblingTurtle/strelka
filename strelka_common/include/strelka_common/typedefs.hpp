/**
 * @file typedefs.hpp
 * Commonly used Eigen matrix types
 *
 */
#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Sparse>

typedef std::chrono::time_point<std::chrono::high_resolution_clock>
    ChronoTimePoint;

template <typename T> using Vec3 = typename Eigen::Matrix<T, 3, 1>;

template <typename T> using Vec4 = typename Eigen::Matrix<T, 4, 1>;

template <typename T> using Vec12 = Eigen::Matrix<T, 12, 1>;

template <typename T> using Mat3 = typename Eigen::Matrix<T, 3, 3>;

template <typename T> using Quat = typename Eigen::Matrix<T, 4, 1>;

template <typename T> using Mat43 = typename Eigen::Matrix<T, 4, 3>;

template <typename T>
using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template <typename T> using SMat = typename Eigen::SparseMatrix<T>;

template <typename T> using DVec = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;

#endif // TYPEDEFS_H