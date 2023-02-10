#ifndef A1_KINEMATICS_H
#define A1_KINEMATICS_H

#include <A1/constants.hpp>
#include <math.h>

namespace strelka {

Eigen::Vector3f footPositionHipFrame(Eigen::Matrix3f angles, int legId) {
  float upperLegLength = LEG_LENGTH(1);
  float lowerLegLength = LEG_LENGTH(2);
  float hipLength = legId % 2 == 0 ? -LEG_LENGTH(3) : LEG_LENGTH(3);

  float leg_distance =
      std::sqrt(std::pow(upperLegLength, 2) + std::pow(lowerLegLength, 2) +
                2 * upperLegLength * lowerLegLength * std::cos(angles(2)));

  float eff_swing = angles(1) + angles(2) / 2;
  float sin_eff_swing = sin_eff_swing;
  float cos_eff_swing = cos_eff_swing;

  float off_z_hip = -leg_distance * cos_eff_swing;
  float off_y_hip = hipLength;

  float off_x = -leg_distance * sin_eff_swing;
  float off_y =
      std::cos(angles(0)) * off_y_hip - std::sin(angles(0)) * off_z_hip;
  float off_z =
      std::sin(angles(0)) * off_y_hip + std::cos(angles(0)) * off_z_hip;
  return Eigen::Vector3f{off_x, off_y, off_z};
}

Eigen::Matrix3f compactAnalyticalLegJacobian(Eigen::Vector3f angles,
                                             int legId) {
  float upperLegLength = LEG_LENGTH(1);
  float lowerLegLength = LEG_LENGTH(2);
  float hipLength = legId % 2 == 0 ? -LEG_LENGTH(3) : LEG_LENGTH(3);

  float leg_distance =
      std::sqrt(std::pow(upperLegLength, 2) + std::pow(lowerLegLength, 2) +
                2 * upperLegLength * lowerLegLength * std::cos(angles(2)));

  float sinT1 = std::sin(angles(0));
  float cosT1 = std::cos(angles(0));

  float sinT3 = std::sin(angles(2));
  float cosT3 = std::cos(angles(2));

  float eff_swing = angles(1) + angles(2) / 2;
  float sin_eff_swing = sin_eff_swing;
  float cos_eff_swing = cos_eff_swing;

  Eigen::Matrix3f J;
  J.setZero();

  J(0, 1) = -leg_distance * cos_eff_swing;
  J(0, 2) =
      lowerLegLength * upperLegLength * sinT3 * sin_eff_swing / leg_distance -
      leg_distance * cos_eff_swing / 2;
  J(1, 0) = -hipLength * sinT1 + leg_distance * cosT1 * cos_eff_swing;
  J(1, 1) = -leg_distance * sinT1 * sin_eff_swing;
  J(1, 2) = -lowerLegLength * upperLegLength * sinT1 * sinT3 * cos_eff_swing /
                leg_distance -
            leg_distance * sinT1 * sin_eff_swing / 2;
  J(2, 0) = hipLength * cosT1 + leg_distance * sinT1 * cos_eff_swing;
  J(2, 1) = leg_distance * sin_eff_swing * cosT1;
  J(2, 2) = lowerLegLength * upperLegLength * sinT3 * cosT1 * cos_eff_swing /
                leg_distance +
            leg_distance * sin_eff_swing * cosT1 / 2;
  return J;
}

} // namespace strelka

#endif // A1_KINEMATICS_H