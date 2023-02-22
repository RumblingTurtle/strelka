#ifndef STRELKA_LOCAL_PLANNER_H
#define STRELKA_LOCAL_PLANNER_H

#include <chrono>
#include <common/A1/constants.hpp>
#include <common/macros.hpp>
#include <control/BodyTrajectoryPlanner.hpp>
#include <control/FootholdPlanner.hpp>
#include <control/MPC.hpp>
#include <control/OldMPC.hpp>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <messages/HighLevelCommand.hpp>
#include <messages/a1_lcm_msgs/WbicCommand.hpp>
#include <robots/UnitreeA1.hpp>

namespace strelka {
namespace control {
class LocalPlanner {
  lcm::LCM lcm;
  GaitScheduler scheduler;
  BodyTrajectoryPlanner bodyPlanner;
  FootholdPlanner footPlanner;
  MPC *mpc;
  Mpc *oldMpc;

  a1_lcm_msgs::HighLevelCommand *highCommand;
  a1_lcm_msgs::WbicCommand *wbicCommand;
  lcm::Subscription *stateSub;
  lcm::Subscription *commandSub;

  const float MPC_DT = 0.02;
  const int horizonSteps = 15;
  float prevTick;

public:
  LocalPlanner();

  ~LocalPlanner();

  void stateHandle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                   const a1_lcm_msgs::RobotState *messageIn);

  void commandHandle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                     const a1_lcm_msgs::HighLevelCommand *commandMsg);

  void processLoop();
};
} // namespace control
} // namespace strelka

#endif // STRELKA_LOCAL_PLANNER_H