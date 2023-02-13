#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <eigen3/Eigen/Core>

const Eigen::Vector3f POSITION_GAINS{100.0, 100.0, 100.0};
const Eigen::Vector3f DAMPING_GAINS{1.0, 2.0, 2.0};

const Eigen::Vector3f STAND_ANGLES{0.0, 0.8, -1.65};
const Eigen::Vector3f INIT_ANGLES{0.25, 1.14, -2.72};

const Eigen::Vector3f LEG_LENGTH{0.0838, 0.2, 0.2};

const Eigen::Vector3f GRAVITY_CONSTANT{0, 0, -9.80665};

const float TRUNK_TO_HIP_OFFSETS[12] = {0.1805, -0.047,  0.0,     0.1805,
                                        0.047,  0.0,     -0.1805, -0.047,
                                        0.0,    -0.1805, 0.047,   0.0};
const float FOOT_RADIUS = 0.02;
const float FOOT_FORCE_THRESHOLD = 10.0;

#endif // CONSTANTS_H