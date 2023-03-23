/**
 * @file kinematics.hpp
 * Kinematics functions for Unitree A1 robot
 */
#ifndef A1_KINEMATICS_H
#define A1_KINEMATICS_H

#include <math.h>
#include <strelka/common/typedefs.hpp>
#include <strelka/robots/A1/constants.hpp>

namespace strelka {
namespace A1 {
namespace kinematics {

/**
 * @brief Given the angle and number of the leg in a given order:
 *
 * 0 Front right (FR)
 * 1 Front left (FL)
 * 2 Rear right (RR)
 * 3 Rear left (RL)
 *
 * computes the position of the foot in the hip frame
 *
 * @param angles Eigen::Matrix<float,3,1> vector representing angles for a
 * single leg
 * @param legId number from 0 to 3 determining direction of the hip link:
 *
 * FR = -1
 * FL = +1
 * RR = -1
 * RL = +1
 *
 * @return Vec3<float> Hip frame position of the foot link
 */
inline Vec3<float> footPositionHipFrame(const Vec3<float> &angles, int legId) {
  assert(legId >= 0 && legId < 4);
  float hipLength = A1::constants::LEG_LENGTH[0] * std::pow(-1, legId + 1);
  float legLength =
      std::sqrt(A1::constants::LEG_LENGTH[1] * A1::constants::LEG_LENGTH[1] +
                A1::constants::LEG_LENGTH[2] * A1::constants::LEG_LENGTH[2] +
                2 * A1::constants::LEG_LENGTH[1] *
                    A1::constants::LEG_LENGTH[2] * std::cos(angles[2]));

  float effSwing = angles[1] + angles[2] / 2;

  float hipOffsetX = -legLength * std::sin(effSwing);
  float hipOffsetZ = -legLength * std::cos(effSwing);
  float hipOffsetY = hipLength;

  float cosTheta0 = std::cos(angles[0]);
  float sinTheta0 = std::sin(angles[0]);

  float xOffset = hipOffsetX;
  float yOffset = cosTheta0 * hipOffsetY - sinTheta0 * hipOffsetZ;
  float zOffset = sinTheta0 * hipOffsetY + cosTheta0 * hipOffsetZ;

  return Vec3<float>{xOffset, yOffset, zOffset};
}

/**
 * @brief Computes jacobian matrix relating joint velocities to end effector
 * velocity for a single foot from  current angle configuration and number of
 * the leg in a given order:
 *
 * 0 Front right (FR)
 * 1 Front left (FL)
 * 2 Rear right (RR)
 * 3 Rear left (RL)
 *
 * @param angles Eigen::Matrix<float,3,1> vector representing angles for a
 * single leg
 * @param legId number from 0 to 3 determining direction of the hip link
 *
 * FR = -1
 * FL = +1
 * RR = -1
 * RL = +1
 *
 * @return Mat3<float> Jacobian matrix
 */
inline Mat3<float> analyticalLegJacobian(const Vec3<float> &angles, int legId) {
  float upperLegLength = A1::constants::LEG_LENGTH(1);
  float lowerLegLength = A1::constants::LEG_LENGTH(2);
  float hipLength = legId % 2 == 0 ? -A1::constants::LEG_LENGTH(0)
                                   : A1::constants::LEG_LENGTH(0);

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

  Mat3<float> J;
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
  return J;
}

/**
 * @brief Computes kinematic reachability of the foot position in the trunk
 * frame by trying to find corresponding angle configuration. LegIds are passed
 * in the following order:
 *
 * 0 Front right (FR)
 * 1 Front left (FL)
 * 2 Rear right (RR)
 * 3 Rear left (RL)
 *
 * @param footPositionTrunkFrame Eigen::Matrix<float,3,1> foot position in trunk
 * frame
 * @param legId number from 0 to 3 determining direction of the hip link
 *
 * FR = -1
 * FL = +1
 * RR = -1
 * RL = +1
 *
 * @return bool true if the foot position is kinematically reachable
 */
inline bool trunkFrameIKCheck(const Vec3<float> &footPositionTrunkFrame,
                              int legId) {
  Vec3<float> footPosHipFrame =
      footPositionTrunkFrame -
      Eigen::Map<const Vec3<float>>(
          A1::constants::TRUNK_TO_HIP_OFFSETS + 3 * legId, 3);

  Vec3<float> legLengths{A1::constants::LEG_LENGTH};
  if (legId % 2 == 0) {
    legLengths(0) = -legLengths(0);
  }

  float cos_val =
      (footPosHipFrame.dot(footPosHipFrame) - legLengths.dot(legLengths)) /
      (2 * legLengths(1) * legLengths(2));

  if (cos_val < -1 || cos_val > 1) {
    return false;
  }

  float theta_knee = -std::acos(cos_val);

  float l =
      std::sqrt(legLengths(1) * legLengths(1) + legLengths(2) * legLengths(2) +
                2 * legLengths(1) * legLengths(2) * std::cos(theta_knee));

  float sin_val = -footPosHipFrame(0) / l;

  if (sin_val < -1 || sin_val > 1) {
    return false;
  }

  float theta_hip = std::asin(sin_val) - theta_knee / 2;

  float c1 = legLengths(0) * footPosHipFrame(1) -
             l * std::cos(theta_hip + theta_knee / 2) * footPosHipFrame(2);

  float s1 = l * std::cos(theta_hip + theta_knee / 2) * footPosHipFrame(1) +
             legLengths(0) * footPosHipFrame(2);

  if (s1 == 0 && c1 == 0) {
    return false;
  }

  return true;
}

} // namespace kinematics
} // namespace A1
} // namespace strelka

#endif // A1_KINEMATICS_H