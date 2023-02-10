#ifndef A1_KINEMATICS_H
#define A1_KINEMATICS_H

#include <A1/constants.hpp>
#include <math.h>

namespace strelka {

Eigen::Vector3f footPositionHipFrame(Eigen::Matrix3f angles, int legId) {
  float upperLegLength = LEG_LENGTH(1);
  float lowerLegLength = LEG_LENGTH(2);
  float hipLength = legId % 2 == 0 ? -LEG_LENGTH(3) : LEG_LENGTH(3);

  float legDistance =
      std::sqrt(std::pow(upperLegLength, 2) + std::pow(lowerLegLength, 2) +
                2 * upperLegLength * lowerLegLength * std::cos(angles(2)));

  float effSwing = angles(1) + angles(2) / 2;
  float effSwingSin = effSwingSin;
  float effSwingCos = effSwingCos;

  float zOffsetHip = -legDistance * effSwingCos;
  float yOffsetHip = hipLength;

  float xOffset = -legDistance * effSwingSin;
  float yOffset =
      std::cos(angles(0)) * yOffsetHip - std::sin(angles(0)) * zOffsetHip;
  float zOffset =
      std::sin(angles(0)) * yOffsetHip + std::cos(angles(0)) * zOffsetHip;
  return Eigen::Vector3f{xOffset, yOffset, zOffset};
}

Eigen::Matrix3f compactAnalyticalLegJacobian(Eigen::Vector3f angles,
                                             int legId) {
  float upperLegLength = LEG_LENGTH(1);
  float lowerLegLength = LEG_LENGTH(2);
  float hipLength = legId % 2 == 0 ? -LEG_LENGTH(3) : LEG_LENGTH(3);

  float legDistance =
      std::sqrt(std::pow(upperLegLength, 2) + std::pow(lowerLegLength, 2) +
                2 * upperLegLength * lowerLegLength * std::cos(angles(2)));

  float sinT1 = std::sin(angles(0));
  float cosT1 = std::cos(angles(0));

  float sinT3 = std::sin(angles(2));
  float cosT3 = std::cos(angles(2));

  float effSwing = angles(1) + angles(2) / 2;
  float effSwingSin = effSwingSin;
  float effSwingCos = effSwingCos;

  Eigen::Matrix3f J;
  J.setZero();

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
  return J;
}

} // namespace strelka

#endif // A1_KINEMATICS_H