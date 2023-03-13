#ifndef A1_LOCAL_PLANNER_H
#define A1_LOCAL_PLANNER_H

#include <chrono>
#include <strelka/common/macros.hpp>
#include <strelka/common/typedefs.hpp>
#include <strelka/control/LocalPlanner.hpp>

#include <lcm/lcm-cpp.hpp>

#include <strelka_messages/HighLevelCommand.hpp>
#include <strelka_messages/a1_lcm_msgs/WbicCommand.hpp>

#include <strelka_robots/A1/UnitreeA1.hpp>
#include <strelka_robots/A1/constants.hpp>

namespace strelka {
namespace control {

class A1LocalPlanner {
  lcm::LCM lcm;

  a1_lcm_msgs::HighLevelCommand *highCommand;
  a1_lcm_msgs::WbicCommand *wbicCommand;
  lcm::Subscription *stateSub;
  lcm::Subscription *commandSub;
  LocalPlanner localPlanner;
  float prevTick;

public:
  A1LocalPlanner(Gait initialGait);

  A1LocalPlanner(FootholdPlanner &footPlanner);
  ~A1LocalPlanner();

  void setupProcessLoop();
  void stateHandle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                   const a1_lcm_msgs::RobotState *messageIn);

  void commandHandle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                     const a1_lcm_msgs::HighLevelCommand *commandMsg);

  bool handle();
  void processLoop();
};
} // namespace control
} // namespace strelka

#endif // A1_LOCAL_PLANNER_H