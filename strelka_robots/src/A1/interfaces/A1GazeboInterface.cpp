#include <strelka_robots/A1/interfaces/A1GazeboInterface.hpp>
namespace strelka {
namespace interfaces {

A1GazeboInterface::MoveToHandle::MoveToHandle(A1GazeboInterface &interface,
                                              const char *stateTopicName)
    : interface(interface), prevTick(-1), stateTopicName(stateTopicName) {}

void A1GazeboInterface::MoveToHandle::run(float moveTime,
                                          const Vec12<float> &desiredAngles) {
  assert(desiredAngles.size() == 12);
  assert(moveTime > 0);

  this->moveTime = moveTime;
  this->timeLeft = moveTime;

  this->desiredAngles = desiredAngles;
  getInitAngles();

  lcm::Subscription *sub =
      lcm.subscribe(stateTopicName, &MoveToHandle::moveHandle, this);

  while (timeLeft > 0 && lcm.handle() == 0) {
  }

  lcm.unsubscribe(sub);
}

Vec12<float> A1GazeboInterface::MoveToHandle::getInitAngles() {
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

void A1GazeboInterface::MoveToHandle::initStartAnglesHandle(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const a1_lcm_msgs::RobotRawState *data) {
  startAngles = Eigen::Map<const Vec12<float>>(data->q, 12);
}

void A1GazeboInterface::MoveToHandle::moveHandle(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const a1_lcm_msgs::RobotRawState *data) {
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

void A1GazeboInterface::sendCommandMessage(const Vec12<float> &command) {
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

void A1GazeboInterface::setTorques(const Vec12<float> &torques) {
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

void A1GazeboInterface::setAngles(const Vec12<float> &q) {
  assert(q.size() == 12);
  for (int motorId = 0; motorId < 12; motorId++) {
    commandMessage.q[motorId] = q[motorId];
    commandMessage.kp[motorId] = A1::constants::POSITION_GAINS[motorId % 3];
    commandMessage.dq[motorId] = 0;
    commandMessage.kd[motorId] = A1::constants::DAMPING_GAINS[motorId % 3];
    commandMessage.tau[motorId] = 0;
  }

  lcm.publish("robot_low_command", &commandMessage);
}

void A1GazeboInterface::setAngles(const Vec12<float> &q,
                                  const Vec12<float> &dq) {
  assert(q.size() == 12);
  assert(dq.size() == 12);
  for (int motorId = 0; motorId < 12; motorId++) {
    commandMessage.q[motorId] = q[motorId];
    commandMessage.kp[motorId] = A1::constants::POSITION_GAINS[motorId % 3];
    commandMessage.dq[motorId] = dq[motorId];
    commandMessage.kd[motorId] = A1::constants::DAMPING_GAINS[motorId % 3];
    commandMessage.tau[motorId] = 0;
  }

  lcm.publish("robot_low_command", &commandMessage);
}

Vec12<float> A1GazeboInterface::getAngles() {
  const Vec12<float> dummyAngles;
  MoveToHandle handle{*this, A1::constants::RAW_STATE_TOPIC_NAME};
  return handle.getInitAngles();
}

void A1GazeboInterface::moveTo(const Vec12<float> &angles, float moveTime) {
  assert(moveTime > 0);
  MoveToHandle moveHandle{*this, A1::constants::RAW_STATE_TOPIC_NAME};
  moveHandle.run(moveTime, angles);
}

void A1GazeboInterface::moveTo(const Vec3<float> &angles, float moveTime) {
  assert(moveTime > 0);
  Vec12<float> moveAngles;
  moveAngles.resize(12);
  for (int motorId = 0; motorId < 12; motorId++) {
    bool rightLeg = motorId % 6 == 0;
    moveAngles[motorId] =
        -angles[motorId % 3] * rightLeg + angles[motorId % 3] * (1 - rightLeg);
  }
  moveTo(moveAngles, moveTime);
}

void A1GazeboInterface::moveToInit(float moveTime) {
  moveTo(A1::constants::INIT_ANGLES, moveTime);
}

void A1GazeboInterface::moveToStand(float moveTime) {
  moveTo(A1::constants::STAND_ANGLES, moveTime);
}
} // namespace interfaces

} // namespace strelka
