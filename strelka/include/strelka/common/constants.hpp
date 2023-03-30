/**
 * @file constants.hpp
 * Common constants
 */
#ifndef STRELKA_COMMON_CONSTANTS_H
#define STRELKA_COMMON_CONSTANTS_H
namespace strelka {
namespace constants {
const float GRAVITY_CONSTANT = -9.80665f;

const char *const RAW_STATE_TOPIC_NAME = "raw_state";
const char *const ROBOT_STATE_TOPIC_NAME = "robot_state";
const char *const WBIC_COMMAND_TOPIC_NAME = "wbic_command";

const char *const HIGH_LEVEL_COMMAND_TOPIC_NAME = "high_level_command";
} // namespace constants
} // namespace strelka
#endif // STRELKA_COMMON_CONSTANTS_H