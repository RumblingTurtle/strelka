#ifndef A1_KINEMATICS_H
#define A1_KINEMATICS_H

#include <common/A1/constants.hpp>
#include <math.h>

namespace strelka {
namespace kinematics {
inline void footPositionHipFrame(const Eigen::Vector3f &angles, int legId,
                                 Eigen::Vector3f &footPosition) {

  float hipLength = LEG_LENGTH[0] * std::pow(-1, legId + 1);
  float legLength =
      std::sqrt(LEG_LENGTH[1] * LEG_LENGTH[1] + LEG_LENGTH[2] * LEG_LENGTH[2] +
                2 * LEG_LENGTH[1] * LEG_LENGTH[2] * std::cos(angles[2]));

  float effSwing = angles[1] + angles[2] / 2;

  float hipOffsetX = -legLength * std::sin(effSwing);
  float hipOffsetZ = -legLength * std::cos(effSwing);
  float hipOffsetY = hipLength;

  float cosTheta0 = std::cos(angles[0]);
  float sinTheta0 = std::sin(angles[0]);

  float xOffset = hipOffsetX;
  float yOffset = cosTheta0 * hipOffsetY - sinTheta0 * hipOffsetZ;
  float zOffset = sinTheta0 * hipOffsetY + cosTheta0 * hipOffsetZ;

  footPosition(0) = xOffset;
  footPosition(1) = yOffset;
  footPosition(2) = zOffset;
}

inline void analyticalLegJacobian(const Eigen::Vector3f &angles, int legId,
                                  Eigen::Matrix3f &J) {
  float upperLegLength = LEG_LENGTH(1);
  float lowerLegLength = LEG_LENGTH(2);
  float hipLength = legId % 2 == 0 ? -LEG_LENGTH(0) : LEG_LENGTH(0);

  float legDistance =
      std::sqrt(std::pow(upperLegLength, 2) + std::pow(lowerLegLength, 2) +
                2 * upperLegLength * lowerLegLength * std::cos(angles(2)));

  float sinT1 = std::sin(angles(0));
  float cosT1 = std::cos(angles(0));

  float sinT3 = std::sin(angles(2));
  float cosT3 = std::cos(angles(2));

  float effSwing = angles(1) + angles(2) / 2;
  float effSwingSin = std::sin(effSwing);
  float effSwingCos = std::cos(effSwing);

  J(0, 0) = 0.0;
  J(0, 1) = -legDistance * effSwingCos;
  J(0, 2) =
      lowerLegLength * upperLegLength * sinT3 * effSwingSin / legDistance -
      legDistance * effSwingCos / 2;
  J(1, 0) = -hipLength * sinT1 + legDistance * cosT1 * effSwingCos;
  J(1, 1) = -legDistance * sinT1 * effSwingSin;
  J(1, 2) = -lowerLegLength * upperLegLength * sinT1 * sinT3 * effSwingCos /
                legDistance -
            legDistance * sinT1 * effSwingSin / 2;
  J(2, 0) = hipLength * cosT1 + legDistance * sinT1 * effSwingCos;
  J(2, 1) = legDistance * effSwingSin * cosT1;
  J(2, 2) = lowerLegLength * upperLegLength * sinT3 * cosT1 * effSwingCos /
                legDistance +
            legDistance * effSwingSin * cosT1 / 2;
}

inline void quatToEulerMatrix(Eigen::Matrix<float, 4, 1> q,
                              Eigen::Matrix3f &output) {
  float w = q(0), x = q(1), y = q(2), z = q(3);

  output(0, 0) = 0.5 - std::pow(y, 2) - std::pow(z, 2);
  output(0, 1) = x * y - w * z;
  output(0, 2) = x * z + w * y;

  output(1, 0) = x * y + w * z;
  output(1, 1) = 0.5 - std::pow(x, 2) - std::pow(z, 2);
  output(1, 2) = y * z - w * x;

  output(2, 0) = x * z - w * y;
  output(2, 1) = y * z + w * x;
  output(2, 2) = 0.5 - std::pow(x, 2) - std::pow(y, 2);

  output *= 2;
}
} // namespace kinematics
} // namespace strelka

#endif // A1_KINEMATICS_H