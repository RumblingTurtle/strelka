#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <strelka_messages/HighLevelCommand.hpp>
#include <strelka_messages/a1_lcm_msgs/HighLevelCommand.hpp>
#include <strelka_robots/A1/constants.hpp>
#include <unistd.h>

void sigHandler(int s) {
  std::cout << "Stopping publisher" << std::endl;
  exit(1);
}

void setupKeyboardInterrupt() {
  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = sigHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

int main() {
  using namespace strelka::messages;
  lcm::LCM lcm;

  if (!lcm.good()) {
    return 1;
  }
  a1_lcm_msgs::HighLevelCommand highCommandMsg{.linearSpeed = {0.2, 0, 0},
                                               .angularVelocity = {0, 0, 0},
                                               .footHeight = 0.08,
                                               .footClearance = 0.002,
                                               .hipOffsets = {0, 0},
                                               .rpy = {0, 0, 0},
                                               .comOffset = {0, 0},
                                               .bodyHeight = 0.26,
                                               .stop = false};

  setupKeyboardInterrupt();
  while (true) {
    lcm.publish(strelka::A1::constants::HIGH_LEVEL_COMMAND_TOPIC_NAME,
                &highCommandMsg);
    usleep(10000);
  }
}