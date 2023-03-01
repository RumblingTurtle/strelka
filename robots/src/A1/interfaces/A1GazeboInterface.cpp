#include <robots/A1/interfaces/A1GazeboInterface.hpp>

namespace strelka {
namespace interfaces {

A1GazeboInterface::MoveToHandle::MoveToHandle(
    float moveTime, const Eigen::VectorXf &desiredAngles,
    A1GazeboInterface &interface)
    : interface(interface), desiredAngles(desiredAngles), firstHandle(true),
      prevTick(-1) {
  assert(desiredAngles.size() == 12);
  assert(moveTime > 0);

  this->moveTime = moveTime;
  this->timeLeft = moveTime;
}

void A1GazeboInterface::MoveToHandle::handle(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const a1_lcm_msgs::RobotRawState *data) {

  if (firstHandle) {
    startAngles = Eigen::Map<const Eigen::VectorXf>(data->q, 12);
    firstHandle = false;
  }

  float t = moveTime - timeLeft;
  float tf = moveTime;

  if (t > tf) {
    return;
  }

  Eigen::VectorXf q, dq;
  q.resize(12);
  dq.resize(12);

  for (int motorId = 0; motorId < 12; motorId++) {
    q(motorId) = 0.5 * (startAngles(motorId) - desiredAngles(motorId)) *
                     std::cos(M_PI * t / tf) +
                 0.5 * (startAngles(motorId) + desiredAngles(motorId));

    dq(motorId) = 0.5 * M_PI * (desiredAngles(motorId) - startAngles(motorId)) *
                  std::sin(M_PI * t / tf) / tf;
  }

  interface.setAngles(q, dq);

  if (prevTick == -1) {
    prevTick = data->tick;
  }

  float dt = data->tick - prevTick;
  prevTick = data->tick;

  timeLeft -= dt;
}

void A1GazeboInterface::sendCommandMessage(const Eigen::VectorXf &command) {
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

void A1GazeboInterface::setTorques(const Eigen::VectorXf &torques) {
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

void A1GazeboInterface::setAngles(const Eigen::VectorXf &q) {
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

void A1GazeboInterface::setAngles(const Eigen::VectorXf &q,
                                  const Eigen::VectorXf &dq) {
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

void A1GazeboInterface::moveTo(const Eigen::VectorXf &angles, float moveTime) {
  assert(angles.size() == 12);
  assert(moveTime > 0);

  MoveToHandle moveHandle{moveTime, angles, *this};

  lcm::Subscription *sub =
      lcm.subscribe("raw_state", &MoveToHandle::handle, &moveHandle);

  while (moveHandle.timeLeft > 0 && lcm.handle() == 0) {
  }

  lcm.unsubscribe(sub);
}

void A1GazeboInterface::moveToInit(float moveTime) {
  assert(moveTime > 0);
  Eigen::VectorXf moveAngles;
  moveAngles.resize(12);
  for (int motorId = 0; motorId < 12; motorId++) {
    bool rightLeg = motorId % 6 == 0;
    moveAngles[motorId] =
        -A1::constants::INIT_ANGLES[motorId % 3] * rightLeg +
        A1::constants::INIT_ANGLES[motorId % 3] * (1 - rightLeg);
  }
  moveTo(moveAngles, moveTime);
}

void A1GazeboInterface::moveToStand(float moveTime) {
  assert(moveTime > 0);
  Eigen::VectorXf moveAngles;
  moveAngles.resize(12);
  for (int motorId = 0; motorId < 12; motorId++) {
    bool rightLeg = motorId % 6 == 0;
    moveAngles[motorId] =
        -A1::constants::STAND_ANGLES[motorId % 3] * rightLeg +
        A1::constants::STAND_ANGLES[motorId % 3] * (1 - rightLeg);
  }
  moveTo(moveAngles, moveTime);
}
} // namespace interfaces

} // namespace strelka
