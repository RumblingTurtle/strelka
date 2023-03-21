#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <strelka/common/constants.hpp>
#include <strelka_lcm_headers/HighLevelCommand.hpp>
#include <strelka_messages/HighLevelCommand.hpp>
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
  strelka_lcm_headers::HighLevelCommand highCommandMsg{
      .linearSpeed = {0.2, 0, 0},
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
    lcm.publish(strelka::constants::HIGH_LEVEL_COMMAND_TOPIC_NAME,
                &highCommandMsg);
    usleep(10000);
  }
}