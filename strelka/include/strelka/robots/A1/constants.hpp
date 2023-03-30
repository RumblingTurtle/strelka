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

const float TRUNK_TO_HIP_OFFSETS[12] = {0.1805, -0.047,  0.0,     0.1805,
                                        0.047,  0.0,     -0.1805, -0.047,
                                        0.0,    -0.1805, 0.047,   0.0};

const float HIP_TO_THIGH_OFFSET[12] = {0, -0.0838, 0, 0, 0.0838, 0,
                                       0, -0.0838, 0, 0, 0.0838, 0};

const Vec3<float> TRUNK_TO_COM_OFFSET{-0.01, 0.002, 0.0};

const Vec3<float> BODY_DIMENSIONS{0.1805 * 2, 0.047 * 2, 0.01675 * 2};
const Vec3<float> LEG_LENGTH{0.0838, 0.2, 0.2};

const float FOOT_RADIUS = 0.02;
const float FOOT_FORCE_THRESHOLD = 10.0;

const Vec3<float> BODY_INERTIA{0.015853, 0.037799, 0.045654};

const Mat3<float> BODY_INERTIA_MATRIX = BODY_INERTIA.asDiagonal();

const float ROBOT_MASS = 12.0;
const float TRUNK_MASS = 5.0;

} // namespace constants
} // namespace A1
} // namespace strelka

#endif // A1_CONSTANTS_H