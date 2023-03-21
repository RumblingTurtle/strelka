#ifndef WBIC_CMD_H
#define WBIC_CMD_H
#include <strelka_lcm_headers/WbicCommand.hpp>

namespace strelka {
namespace messages {

/**
 * @brief Command object passed to the Whole-body impulse controller
 *  Every messsage entry should be in world frame
 */
class WBICCommand {
  Vec4<float> _desiredContactState;
  Vec3<float> _desiredBodyRPY;
  Vec3<float> _desiredBodyAngularVelocity;
  Vec3<float> _desiredBodyPosition;
  Vec3<float> _desiredBodyVelocity;
  Vec3<float> _desiredBodyAcceleration;
  Vec12<float> _desiredFootPosition;
  Vec12<float> _desiredFootVelocity;
  Vec12<float> _desiredFootAcceleration;
  Vec12<float> _desiredFootForceWorld;

public:
  WBICCommand(const strelka_lcm_headers::WbicCommand *commandMsg) {
    _desiredContactState =
        Eigen::Map<const Vec4<float>>(commandMsg->footState, 4);
    _desiredBodyRPY = Eigen::Map<const Vec3<float>>(commandMsg->rpy, 3);
    _desiredBodyAngularVelocity =
        Eigen::Map<const Vec3<float>>(commandMsg->angularVelocity, 3);

    _desiredBodyPosition = Eigen::Map<const Vec3<float>>(commandMsg->pBody, 3);
    _desiredBodyVelocity = Eigen::Map<const Vec3<float>>(commandMsg->vBody, 3);
    _desiredBodyAcceleration =
        Eigen::Map<const Vec3<float>>(commandMsg->aBody, 3);

    _desiredFootPosition =
        Eigen::Map<const Vec12<float>>(commandMsg->pFoot, 12);
    _desiredFootVelocity =
        Eigen::Map<const Vec12<float>>(commandMsg->vFoot, 12);
    _desiredFootAcceleration =
        Eigen::Map<const Vec12<float>>(commandMsg->aFoot, 12);

    _desiredFootForceWorld =
        Eigen::Map<const Vec12<float>>(commandMsg->mpcForces, 12);
  }

  Vec4<float> desiredContactState() { return _desiredContactState; }
  Vec3<float> desiredBodyRPY() { return _desiredBodyRPY; }
  Vec3<float> desiredBodyAngularVelocity() {
    return _desiredBodyAngularVelocity;
  }
  Vec3<float> desiredBodyPosition() { return _desiredBodyPosition; }
  Vec3<float> desiredBodyVelocity() { return _desiredBodyVelocity; }
  Vec3<float> desiredBodyAcceleration() { return _desiredBodyAcceleration; }
  Vec12<float> desiredFootPosition() { return _desiredFootPosition; }
  Vec12<float> desiredFootVelocity() { return _desiredFootVelocity; }
  Vec12<float> desiredFootAcceleration() { return _desiredFootAcceleration; }
  Vec12<float> desiredFootForceWorld() { return _desiredFootForceWorld; }
};

} // namespace messages
} // namespace strelka

#endif // WBIC_CMD_H