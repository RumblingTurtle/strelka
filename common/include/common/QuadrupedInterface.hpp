
#ifndef QUADRUPED_INTERFACE_H
#define QUADRUPED_INTERFACE_H
#include <common/typedefs.hpp>
namespace strelka {
namespace interfaces {
/**
 * @brief Generic interface for any 12 motor robot used to send desired
 * angles,joint velocities and torques
 *
 */
class QuadrupedInterface {
public:
  virtual void sendCommandMessage(const Vec12<float> &command) = 0;
  virtual void setTorques(const Vec12<float> &torques) = 0;

  virtual void setAngles(const Vec12<float> &q) = 0;
  virtual void setAngles(const Vec12<float> &q, const Vec12<float> &dq) = 0;

  /**
   * @brief Get current angle configuration
   *
   * @return Vec12<float> current motor angles
   */
  virtual Vec12<float> getAngles() = 0;

  /**
   * @brief Moves the robot to specified angle configuration over moveTime
   * seconds
   *
   * @param angles 12 desired motor angles
   * @param moveTime Time to move the robot from current to specified
   * configuration
   */
  virtual void moveTo(const Vec12<float> &angles, float moveTime) = 0;

  /**
   * @brief Moves the robot to initial configuration over moveTime seconds
   * Initial configuration might be lying on the ground or calibration
   * configuration
   *
   * @param moveTime Time to move the robot from current to initial
   * configuration
   */
  virtual void moveToInit(float moveTime = 3.0) = 0;

  /**
   * @brief Moves the robot to stand configuration over moveTime seconds
   *
   * @param moveTime Time to move the robot from current to stand configuration
   */
  virtual void moveToStand(float moveTime = 3.0) = 0;
};
} // namespace interfaces
} // namespace strelka

#endif // QUADRUPED_INTERFACE_H