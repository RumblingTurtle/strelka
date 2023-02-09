
#ifndef QUADRUPED_INTERFACE_H
#define QUADRUPED_INTERFACE_H

namespace strelka {
class QuadrupedInterface {
public:
  virtual void sendCommandMessage(const Eigen::VectorXf &command) = 0;
  virtual void setTorques(const Eigen::VectorXf &torques) = 0;

  virtual void setAngles(const Eigen::VectorXf &q) = 0;
  virtual void setAngles(const Eigen::VectorXf &q,
                         const Eigen::VectorXf &dq) = 0;

  virtual void moveTo(const Eigen::VectorXf &angles, float moveTime) = 0;
  virtual void moveToInit(float moveTime = 3.0) = 0;
  virtual void moveToStand(float moveTime = 3.0) = 0;
};
} // namespace strelka

#endif // QUADRUPED_INTERFACE_H