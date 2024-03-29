#ifndef HIGH_CMD_H
#define HIGH_CMD_H
#include <strelka/common/typedefs.hpp>
#include <strelka_lcm_headers/HighLevelCommand.hpp>
namespace strelka {
namespace messages {

/**
 * @brief Eigen wrapper for a high level command to a Quadruped robot. Possible
 * command parameters are as follows:
 *
 *  - Desired angular/linear velocites in body frame
 *  - Desired roll pitch yaw
 *  - Desired offset of the trunk from the initial position. Usually used while
 * standing to move the trunk around the inital position
 *  - Desired body height
 *  - Desired foot height while swinging
 */
class HighLevelCommand {
  Vec3<float> _desiredAngularVelocityBodyFrame;
  Vec3<float> _desiredLinearVelocityBodyFrame;
  Vec3<float> _desiredRPY;
  Vec3<float> _desiredComOffset;
  float _desiredFootHeight;
  float _desiredBodyHeight;
  float _footClearance;
  float _ignoreVelocity;
  /*


    float hipOffsets[2];

    int8_t stop;

    TODO:... what else? Gait type?
  */

public:
  HighLevelCommand(const strelka_lcm_headers::HighLevelCommand *commandMsg)
      : _ignoreVelocity(false) {
    _desiredAngularVelocityBodyFrame =
        Eigen::Map<const Vec3<float>>(commandMsg->angularVelocity, 3);
    _desiredLinearVelocityBodyFrame =
        Eigen::Map<const Vec3<float>>(commandMsg->linearSpeed, 3);
    _desiredRPY = Eigen::Map<const Vec3<float>>(commandMsg->rpy, 3);

    _desiredComOffset(0) = commandMsg->comOffset[0];
    _desiredComOffset(1) = commandMsg->comOffset[1];
    _desiredComOffset(2) = 0;
    _desiredBodyHeight = commandMsg->bodyHeight;
    _desiredFootHeight = commandMsg->footHeight;
    _footClearance = commandMsg->footClearance;
  }

  Vec3<float> desiredAngularVelocityBodyFrame() {
    if (_ignoreVelocity) {
      return Vec3<float>{0, 0, 0};
    }
    return _desiredAngularVelocityBodyFrame;
  };
  Vec3<float> desiredLinearVelocityBodyFrame() {
    if (_ignoreVelocity) {
      return Vec3<float>{0, 0, 0};
    }
    return _desiredLinearVelocityBodyFrame;
  };
  Vec3<float> desiredRPY() { return _desiredRPY; };
  Vec3<float> desiredComOffset() { return _desiredComOffset; };
  float desiredBodyHeight() { return _desiredBodyHeight; };
  float desiredFootHeight() { return _desiredFootHeight; };
  float footClearance() { return _footClearance; }

  void setIgnoreDesiredBodyVelocity(bool ignoreVelocity) {
    _ignoreVelocity = ignoreVelocity;
  }
  static HighLevelCommand
  makeDummyCommandMessage(float desiredVelocityX = 0.0,
                          float desiredVelocityYaw = 0.0) {
    strelka_lcm_headers::HighLevelCommand highCommandMsg{
        .linearSpeed = {desiredVelocityX, 0, 0},
        .angularVelocity = {0, 0, desiredVelocityYaw},
        .footHeight = 0.08,
        .footClearance = 0.002,
        .hipOffsets = {0, 0},
        .rpy = {0, 0, 0},
        .comOffset = {0, 0},
        .bodyHeight = 0.26,
        .stop = false};

    return HighLevelCommand(&highCommandMsg);
  }
};

} // namespace messages
} // namespace strelka

#endif // HIGH_CMD_H