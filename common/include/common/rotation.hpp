
#ifndef A1_ROTATION_H
#define A1_ROTATION_H

#include <common/typedefs.hpp>
#include <math.h>

namespace strelka {
namespace rotation {

inline void quat2rot(Vec4<float> q, Mat3<float> &output) {
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

inline void quat2euler(Vec4<float> q, Vec3<float> &output) {

  output(0) = std::atan2((2 * (q[0] * q[1] + q[2] * q[3])),
                         (1 - 2 * (q[1] * q[1] + q[2] * q[2])));
  output(1) = std::asin(2 * (q[0] * q[2] - q[3] * q[1]));
  output(2) = std::atan2((2 * (q[0] * q[3] + q[1] * q[2])),
                         (1 - 2 * (q[2] * q[2] + q[3] * q[3])));
}

inline void rpy2rot(Vec3<float> rpy, Mat3<float> &output) {
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

inline void rpy2rot(Vec3<double> rpy, Mat3<double> &output) {
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
