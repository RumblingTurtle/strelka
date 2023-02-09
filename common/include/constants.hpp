#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <eigen3/Eigen/Core>

const Eigen::Vector3f POSITION_GAINS{100.0, 100.0, 100.0};
const Eigen::Vector3f DAMPING_GAINS{1.0, 2.0, 2.0};

const Eigen::Vector3f STAND_ANGLES{0.0, 0.8, -1.65};
const Eigen::Vector3f INIT_ANGLES{0.25, 1.14, -2.72};

#endif // CONSTANTS_H