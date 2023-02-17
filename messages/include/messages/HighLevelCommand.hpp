#ifndef HIGH_CMD_H
#define HIGH_CMD_H
#include <messages/a1_lcm_msgs/HighLevelCommand.hpp>

namespace strelka {
namespace messages {

class HighLevelCommand {
  Vec3<float> _desiredAngularVelocityBodyFrame;
  Vec3<float> _desiredLinearVelocityBodyFrame;
  Vec3<float> _desiredRPY;
  Vec3<float> _desiredComOffset;
  float _desiredFootHeight;
  float _desiredBodyHeight;
  /*
    float linearSpeed[3];

    float angularVelocity[3];

    float footHeight;

    float footClearance;

    float hipOffsets[2];

    float rpy[3];

    float comOffset[2];

    float bodyHeight;

    int8_t stop;
    */

public:
  HighLevelCommand(const a1_lcm_msgs::HighLevelCommand *commandMsg) {
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
  }

  Vec3<float> desiredAngularVelocityBodyFrame() {
    return _desiredAngularVelocityBodyFrame;
  };
  Vec3<float> desiredLinearVelocityBodyFrame() {
    return _desiredLinearVelocityBodyFrame;
  };
  Vec3<float> desiredRPY() { return _desiredRPY; };
  Vec3<float> desiredComOffset() { return _desiredComOffset; };
  float desiredBodyHeight() { return _desiredBodyHeight; };
  float desiredFootHeight() { return _desiredFootHeight; };

  static HighLevelCommand makeDummyCommandMessage(float desiredVelocityX) {
    a1_lcm_msgs::HighLevelCommand highCommandMsg{
        .linearSpeed = {desiredVelocityX, 0, 0},
        .angularVelocity = {0, 0, 0},
        .footHeight = 0.12,
        .footClearance = 0.02,
        .hipOffsets = {0, 0},
        .rpy = {0, 0, 0},
        .comOffset = {0, 0},
        .bodyHeight = 0.27,
        .stop = false};

    return HighLevelCommand(&highCommandMsg);
  }
};

} // namespace messages
} // namespace strelka

#endif // HIGH_CMD_H