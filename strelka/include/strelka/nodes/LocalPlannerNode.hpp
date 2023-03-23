#ifndef LOCAL_PLANNER_NODE_H
#define LOCAL_PLANNER_NODE_H

#include <chrono>
#include <strelka/common/macros.hpp>
#include <strelka/common/typedefs.hpp>
#include <strelka/control/LocalPlanner.hpp>

#include <lcm/lcm-cpp.hpp>

#include <strelka_lcm_headers/RobotState.hpp>
#include <strelka_lcm_headers/WbicCommand.hpp>
#include <strelka_messages/HighLevelCommand.hpp>

#include <strelka/robots/Robots.hpp>

namespace strelka {
namespace control {

template <class RobotClass> class LocalPlannerNode {
  RobotClass robotInstance;
  lcm::LCM lcm;

  strelka_lcm_headers::HighLevelCommand *highCommand;
  strelka_lcm_headers::WbicCommand *wbicCommand;
  lcm::Subscription *stateSub;
  lcm::Subscription *commandSub;
  LocalPlanner localPlanner;
  float prevTick;

  ChronoTimePoint lastCommandTimestamp;
  bool firstCommandRecieved;

public:
  static constexpr float COMMAND_TIMEOUT_SECONDS = 0.5;

  LocalPlannerNode(Gait initialGait, float stepDt, int horizonSteps,
                   float heightFilterCutoffFrequency,
                   float pitchFilterCutoffFrequency,
                   bool updateFootholdsContinuously);

  LocalPlannerNode(std::shared_ptr<FootholdPlanner> footPlanner, float stepDt,
                   int horizonSteps, float heightFilterCutoffFrequency,
                   float pitchFilterCutoffFrequency);

  ~LocalPlannerNode();

  LocalPlanner &getLocalPlanner();
  void setupProcessLoop();
  void stateHandle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                   const strelka_lcm_headers::RobotState *messageIn);

  void commandHandle(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                     const strelka_lcm_headers::HighLevelCommand *commandMsg);

  bool handle();
  void processLoop();
};
} // namespace control
} // namespace strelka

#endif // LOCAL_PLANNER_NODE_H