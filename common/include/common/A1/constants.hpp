#ifndef A1_CONSTANTS_H
#define A1_CONSTANTS_H

#include <common/typedefs.hpp>
namespace strelka {
namespace constants {
namespace A1 {
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
const double MPC_WEIGHTS[13] = {1.0, 1.0, 0.0, 0.0, 0.0, 50.0, 0.0f,
                                0.0, 1.0, 1.0, 1.0, 0.0, 0.0};

const double MPC_CONSTRAINT_MAX_SCALE = 10;
const double MPC_CONSTRAINT_MIN_SCALE = 0.1;
const double MPC_ALPHA = 1e-5;
const double MPC_BODY_MASS = 12.8;
const Vec4<double> MPC_FRICTION_COEFFS{0.45, 0.45, 0.45, 0.45};
const Vec3<double> MPC_BODY_INERTIA{0.15, 0.340, 0.36};

} // namespace A1
} // namespace constants
} // namespace strelka

#endif // A1_CONSTANTS_H