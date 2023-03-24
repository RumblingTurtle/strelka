#ifndef UNITREE_A1_H
#define UNITREE_A1_H

#include <strelka/robots/A1/constants.hpp>
#include <strelka/robots/A1/kinematics.hpp>

#include <exception>
#include <strelka/common/constants.hpp>
#include <strelka/common/macros.hpp>
#include <strelka/common/rotation.hpp>
#include <strelka/common/typedefs.hpp>
#include <strelka/robots/Robot.hpp>

namespace strelka {
namespace robots {

/**
 * @brief Example of Robot interface implementation using Unitree A1 robot
 *
 */
class UnitreeA1 : public Robot {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Default constructed instance is used for constants
  explicit UnitreeA1();
  UnitreeA1(const strelka_lcm_headers::RobotRawState *rawStateMessage);
  UnitreeA1(const strelka_lcm_headers::RobotState *robotStateMessage);

  bool worldFrameIKCheck(Vec3<float> footPositionWorldFrame,
                         int legId) override;

  Mat3<float> footJacobianImpl(int legId) override;
  Vec3<float> footPositionTrunkFrameImpl(int legId) override;
  bool estimateContactImpl(int legId) override;

  const float footRadius() override;

  const float trunkMass() override;
  const float robotMass() override;

  Vec3<float> bodyToComOffset() override;

  Vec3<float> bodyDimensions() override;
  Vec3<float> legDimensions() override;

  Vec3<float> positionGains() override;
  Vec3<float> dampingGains() override;

  Vec3<float> initAngles() override;
  Vec3<float> standAngles() override;

  Mat3<float> rotationalInertia() override;

  Vec3<float> trunkToHipOffset(int legId) override;
  Vec3<float> trunkToThighOffset(int legId) override;

  static UnitreeA1 &createDummyA1RobotWithStateEstimates();
  static UnitreeA1 &createDummyA1RobotWithRawState(
      Vec3<float> motorAngles = A1::constants::STAND_ANGLES);
};

} // namespace robots
} // namespace strelka
#endif // UNITREE_A1_H