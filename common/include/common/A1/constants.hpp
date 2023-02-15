#ifndef A1_CONSTANTS_H
#define A1_CONSTANTS_H

#include <eigen3/Eigen/Core>

namespace strelka {
namespace constants {
namespace A1 {
const Eigen::Vector3f POSITION_GAINS{100.0, 100.0, 100.0};
const Eigen::Vector3f DAMPING_GAINS{1.0, 2.0, 2.0};

const Eigen::Vector3f STAND_ANGLES{0.0, 0.8, -1.65};
const Eigen::Vector3f INIT_ANGLES{0.25, 1.14, -2.72};

const Eigen::Vector3f LEG_LENGTH{0.0838, 0.2, 0.2};

const Eigen::Vector3f TRUNK_TO_COM_OFFSET{-0.01, 0.002, 0.0};

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
} // namespace A1
} // namespace constants
} // namespace strelka

#endif // A1_CONSTANTS_H