#include <interfaces/GazeboInterface.hpp>

namespace strelka {

GazeboInterface::MoveToHandle::MoveToHandle(
    float moveTime, const Eigen::VectorXf &desiredAngles,
    GazeboInterface &interface)
    : interface(interface), desiredAngles(desiredAngles), firstHandle(true),
      prevTick(-1) {
  assert(desiredAngles.size() == 12);
  assert(moveTime > 0);

  this->moveTime = moveTime;
  this->timeLeft = moveTime;
}

void GazeboInterface::MoveToHandle::handle(
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

void GazeboInterface::sendCommandMessage(const Eigen::VectorXf &command) {
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

void GazeboInterface::setTorques(const Eigen::VectorXf &torques) {
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

void GazeboInterface::setAngles(const Eigen::VectorXf &q) {
  assert(q.size() == 12);
  for (int motorId = 0; motorId < 12; motorId++) {
    commandMessage.q[motorId] = q[motorId];
    commandMessage.kp[motorId] = POSITION_GAINS[motorId % 3];
    commandMessage.dq[motorId] = 0;
    commandMessage.kd[motorId] = DAMPING_GAINS[motorId % 3];
    commandMessage.tau[motorId] = 0;
  }

  lcm.publish("robot_low_command", &commandMessage);
}

void GazeboInterface::setAngles(const Eigen::VectorXf &q,
                                const Eigen::VectorXf &dq) {
  assert(q.size() == 12);
  assert(dq.size() == 12);
  for (int motorId = 0; motorId < 12; motorId++) {
    commandMessage.q[motorId] = q[motorId];
    commandMessage.kp[motorId] = POSITION_GAINS[motorId % 3];
    commandMessage.dq[motorId] = dq[motorId];
    commandMessage.kd[motorId] = DAMPING_GAINS[motorId % 3];
    commandMessage.tau[motorId] = 0;
  }

  lcm.publish("robot_low_command", &commandMessage);
}

void GazeboInterface::moveTo(const Eigen::VectorXf &angles, float moveTime) {
  assert(angles.size() == 12);
  assert(moveTime > 0);

  MoveToHandle moveHandle{moveTime, angles, *this};

  lcm::Subscription *sub =
      lcm.subscribe("raw_state", &MoveToHandle::handle, &moveHandle);

  while (moveHandle.timeLeft > 0 && lcm.handle() == 0) {
  }

  lcm.unsubscribe(sub);
}

void GazeboInterface::moveToInit(float moveTime) {
  assert(moveTime > 0);
  Eigen::VectorXf moveAngles;
  moveAngles.resize(12);
  for (int motorId = 0; motorId < 12; motorId++) {
    bool rightLeg = motorId % 6 == 0;
    moveAngles[motorId] = -INIT_ANGLES[motorId % 3] * rightLeg +
                          INIT_ANGLES[motorId % 3] * (1 - rightLeg);
  }
  moveTo(moveAngles, moveTime);
}

void GazeboInterface::moveToStand(float moveTime) {
  assert(moveTime > 0);
  Eigen::VectorXf moveAngles;
  moveAngles.resize(12);
  for (int motorId = 0; motorId < 12; motorId++) {
    bool rightLeg = motorId % 6 == 0;
    moveAngles[motorId] = -STAND_ANGLES[motorId % 3] * rightLeg +
                          STAND_ANGLES[motorId % 3] * (1 - rightLeg);
  }
  moveTo(moveAngles, moveTime);
}

} // namespace strelka
