/**
 * @file constants.hpp
 * Constants for Unitree A1 robot
 *
 */
#ifndef A1_CONSTANTS_H
#define A1_CONSTANTS_H

#include <strelka/common/typedefs.hpp>
namespace strelka {
namespace A1 {
namespace constants {

const Vec3<float> POSITION_GAINS{100.0, 100.0, 100.0};
const Vec3<float> DAMPING_GAINS{1.0, 2.0, 2.0};

const Vec3<float> STAND_ANGLES{0.0, 0.8, -1.65};
const Vec3<float> INIT_ANGLES{0.25, 1.14, -2.72};

const Vec3<float> LEG_LENGTH{0.0838, 0.2, 0.2};

const Vec3<float> TRUNK_TO_COM_OFFSET{-0.01, 0.002, 0.0};

const float TRUNK_TO_HIP_OFFSETS[12] = {0.1805, -0.047,  0.0,     0.1805,
                                        0.047,  0.0,     -0.1805, -0.047,
                                        0.0,    -0.1805, 0.047,   0.0};

const float HIP_TO_THIGH_OFFSET[12] = {0, -0.0838, 0, 0, 0.0838, 0,
                                       0, -0.0838, 0, 0, 0.0838, 0};

const float TRUNK_TO_THIGH_OFFSETS[12] = {0.1805, -0.1308, 0.0,     0.1805,
                                          0.1308, 0.0,     -0.1805, -0.1308,
                                          0.0,    -0.1805, 0.1308,  0.0};

const float FOOT_RADIUS = 0.02;
const float FOOT_FORCE_THRESHOLD = 10.0;

const Vec3<float> MPC_BODY_INERTIA{0.15, 0.34, 0.36};
const float MPC_BODY_MASS = 12.8;

const char *const RAW_STATE_TOPIC_NAME = "raw_state";
const char *const ROBOT_STATE_TOPIC_NAME = "robot_state";
const char *const GAZEBO_STATE_TOPIC_NAME = "gazebo_state";
const char *const WBIC_COMMAND_TOPIC_NAME = "wbic_command";

const char *const HIGH_LEVEL_COMMAND_TOPIC_NAME = "high_level_command";

} // namespace constants
} // namespace A1
} // namespace strelka

#endif // A1_CONSTANTS_H