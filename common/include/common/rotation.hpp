
/**
 * @brief Rotation utility functions
 *
 */

#ifndef A1_ROTATION_H
#define A1_ROTATION_H

#include <common/typedefs.hpp>
#include <math.h>

namespace strelka {
namespace rotation {

/**
 * @brief Constructs rotation matrix from scalar-first quaternion
 *
 * @param q Input quaternion Eigen::Matrix<ScalarType,4,1>
 * @param output Rotation matrix Eigen::Matrix<ScalarType,3,3>
 */
template <typename T, typename T2>
void quat2rot(const Eigen::MatrixBase<T> &q, Eigen::MatrixBase<T2> &output) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                "Must have 4x1 vector");
  static_assert(T2::ColsAtCompileTime == 3 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 matrix");

  float w = q(0), x = q(1), y = q(2), z = q(3);

  output(0, 0) = 0.5 - y * y - z * z;
  output(0, 1) = x * y - w * z;
  output(0, 2) = x * z + w * y;

  output(1, 0) = x * y + w * z;
  output(1, 1) = 0.5 - x * x - z * z;
  output(1, 2) = y * z - w * x;

  output(2, 0) = x * z - w * y;
  output(2, 1) = y * z + w * x;
  output(2, 2) = 0.5 - x * x - y * y;

  output *= 2;
}

/**
 * @brief Compute Roll Pitch Yaw angles from scalar-first quaternion
 *
 * @param q Input quaternion Eigen::Matrix<ScalarType,4,1>
 * @return Euler angles
 */
template <typename T>
Vec3<typename T::Scalar> quat2euler(const Eigen::MatrixBase<T> &q) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                "Must have 4x1 vector");

  float roll = std::atan2((2 * (q[0] * q[1] + q[2] * q[3])),
                          (1 - 2 * (q[1] * q[1] + q[2] * q[2])));
  float pitch = std::asin(2 * (q[0] * q[2] - q[3] * q[1]));
  float yaw = std::atan2((2 * (q[0] * q[3] + q[1] * q[2])),
                         (1 - 2 * (q[2] * q[2] + q[3] * q[3])));

  return Vec3<typename T::Scalar>{roll, pitch, yaw};
}

/**
 * @brief
 *
 * @tparam T2
 * @param rpy Euler angles Eigen::Matrix<ScalarType,3,1>
 * @param output Rotation matrix Eigen::Matrix<ScalarType,3,3>
 */
template <typename T, typename T2>
void rpy2rot(const Eigen::MatrixBase<T> &rpy, Eigen::MatrixBase<T2> &output) {
  static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 1 ||
                    T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                "Must have 3x1 vector");
  static_assert(T2::ColsAtCompileTime == 3 && T2::RowsAtCompileTime == 3,
                "Must have 3x3 matrix");

  float c_r = std::cos(rpy(0)), c_p = std::cos(rpy(1)), c_y = std::cos(rpy(2)),
        s_r = std::sin(rpy(0)), s_p = std::sin(rpy(1)), s_y = std::sin(rpy(2));

  output(0, 0) = c_y * c_p;
  output(0, 1) = c_y * s_p * s_r - s_y * c_r;
  output(0, 2) = c_y * s_p * c_r + s_y * s_r;

  output(1, 0) = s_y * c_p;
  output(1, 1) = s_y * s_p * s_r + c_y * c_r;
  output(1, 2) = s_y * s_p * c_r - c_y * s_r;

  output(2, 0) = -s_p;
  output(2, 1) = c_p * s_r;
  output(2, 2) = c_p * c_r;
}

} // namespace rotation
} // namespace strelka

#endif // A1_ROTATION_H
