#include <strelka/nodes/MoveToInterface.hpp>

namespace strelka {
namespace interfaces {

template <class RobotClass>
MoveToInterface<RobotClass>::MoveToHandle::MoveToHandle(
    MoveToInterface &interface, const char *stateTopicName)
    : interface(interface), prevTick(-1), stateTopicName(stateTopicName) {}

template <class RobotClass>
void MoveToInterface<RobotClass>::MoveToHandle::run(
    float moveTime, const Vec12<float> &desiredAngles) {
  assert(desiredAngles.size() == 12);
  assert(moveTime > 0);

  this->moveTime = moveTime;
  this->timeLeft = moveTime;

  this->desiredAngles = desiredAngles;
  getStartAngles();

  lcm::Subscription *sub =
      lcm.subscribe(stateTopicName, &MoveToHandle::moveHandle, this);

  while (timeLeft > 0 && lcm.handle() == 0) {
  }

  lcm.unsubscribe(sub);
}

template <class RobotClass>
Vec12<float> MoveToInterface<RobotClass>::MoveToHandle::getStartAngles() {
  lcm::Subscription *sub =
      lcm.subscribe(stateTopicName, &MoveToHandle::initStartAnglesHandle, this);
  sub->setQueueCapacity(1);

  bool topicNotPublishing = lcm.handleTimeout(500) <= 0;

  if (topicNotPublishing) {
    lcm.unsubscribe(sub);
    throw RobotStateTopicDoesNotExist();
  }

  lcm.handle();
  lcm.unsubscribe(sub);
  return startAngles;
}

template <class RobotClass>
void MoveToInterface<RobotClass>::MoveToHandle::initStartAnglesHandle(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const strelka_lcm_headers::RobotRawState *data) {
  startAngles = Eigen::Map<const Vec12<float>>(data->q, 12);
}

template <class RobotClass>
void MoveToInterface<RobotClass>::MoveToHandle::moveHandle(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const strelka_lcm_headers::RobotRawState *data) {
  Vec12<float> q, dq;

  if (timeLeft <= 0.0) {
    dq.setZero();
    interface.setAngles(desiredAngles, dq);
    return;
  }

  if (prevTick == -1) {
    prevTick = data->tick;
  }

  float dt = data->tick - prevTick;
  prevTick = data->tick;
  timeLeft -= dt;

  float t = moveTime - timeLeft;
  float tf = moveTime;

  for (int motorId = 0; motorId < 12; motorId++) {
    q(motorId) = 0.5 * (startAngles(motorId) - desiredAngles(motorId)) *
                     std::cos(M_PI * t / tf) +
                 0.5 * (startAngles(motorId) + desiredAngles(motorId));

    dq(motorId) = 0.5 * M_PI * (desiredAngles(motorId) - startAngles(motorId)) *
                  std::sin(M_PI * t / tf) / tf;
  }

  interface.setAngles(q, dq);
}

template <class RobotClass>
void MoveToInterface<RobotClass>::sendCommandMessage(
    const Vec12<float> &command) {
  assert(command.size() == 60);
  for (int motorId = 0; motorId < 12; motorId++) {
    commandMessage.q[motorId] = command[5 * motorId];
    commandMessage.kp[motorId] = command[5 * motorId + 1];
    commandMessage.dq[motorId] = command[5 * motorId + 2];
    commandMessage.kd[motorId] = command[5 * motorId + 3];
    commandMessage.tau[motorId] = command[5 * motorId + 4];
  }

  lcm.publish("robot_low_command", &commandMessage);
}

template <class RobotClass>
void MoveToInterface<RobotClass>::setTorques(const Vec12<float> &torques) {
  assert(torques.size() == 12);
  for (int motorId = 0; motorId < 12; motorId++) {
    commandMessage.q[motorId] = 0;
    commandMessage.kp[motorId] = 0;
    commandMessage.dq[motorId] = 0;
    commandMessage.kd[motorId] = 0;
    commandMessage.tau[motorId] = torques[motorId];
  }

  lcm.publish("robot_low_command", &commandMessage);
}

template <class RobotClass>
void MoveToInterface<RobotClass>::setAngles(const Vec12<float> &q) {
  assert(q.size() == 12);
  Vec3<float> KP = robotInstance.positionGains();
  Vec3<float> KD = robotInstance.dampingGains();
  for (int motorId = 0; motorId < 12; motorId++) {
    commandMessage.q[motorId] = q[motorId];
    commandMessage.kp[motorId] = KP[motorId % 3];
    commandMessage.dq[motorId] = 0;
    commandMessage.kd[motorId] = KD[motorId % 3];
    commandMessage.tau[motorId] = 0;
  }

  lcm.publish("robot_low_command", &commandMessage);
}

template <class RobotClass>
void MoveToInterface<RobotClass>::setAngles(const Vec12<float> &q,
                                            const Vec12<float> &dq) {
  assert(q.size() == 12);
  assert(dq.size() == 12);
  Vec3<float> KP = robotInstance.positionGains();
  Vec3<float> KD = robotInstance.dampingGains();
  for (int motorId = 0; motorId < 12; motorId++) {
    commandMessage.q[motorId] = q[motorId];
    commandMessage.kp[motorId] = KP[motorId % 3];
    commandMessage.dq[motorId] = dq[motorId];
    commandMessage.kd[motorId] = KD[motorId % 3];
    commandMessage.tau[motorId] = 0;
  }

  lcm.publish("robot_low_command", &commandMessage);
}

template <class RobotClass>
Vec12<float> MoveToInterface<RobotClass>::getAngles() {
  const Vec12<float> dummyAngles;
  MoveToHandle handle{*this, constants::RAW_STATE_TOPIC_NAME};
  return handle.getStartAngles();
}

template <class RobotClass>
void MoveToInterface<RobotClass>::moveTo(const Vec12<float> &angles,
                                         float moveTime) {
  assert(moveTime > 0);
  MoveToHandle moveHandle{*this, constants::RAW_STATE_TOPIC_NAME};
  moveHandle.run(moveTime, angles);
}

template <class RobotClass>
void MoveToInterface<RobotClass>::moveTo(const Vec3<float> &angles,
                                         float moveTime) {
  assert(moveTime > 0);
  Vec12<float> moveAngles;
  for (int motorId = 0; motorId < 12; motorId++) {
    bool rightLeg = motorId % 6 == 0;
    moveAngles[motorId] =
        -angles[motorId % 3] * rightLeg + angles[motorId % 3] * (1 - rightLeg);
  }
  moveTo(moveAngles, moveTime);
}

template <class RobotClass>
void MoveToInterface<RobotClass>::moveToInit(float moveTime) {
  moveTo(robotInstance.initAngles(), moveTime);
}

template <class RobotClass>
void MoveToInterface<RobotClass>::moveToStand(float moveTime) {
  moveTo(robotInstance.standAngles(), moveTime);
}
} // namespace interfaces
#define MOVE_TO_INTERFACE_HEADER
#include <strelka/robots/RobotRegistry.hpp>
} // namespace strelka
