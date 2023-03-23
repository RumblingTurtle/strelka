/**
 * @file trajectory.hpp
 * Trajectory utilities
 */
#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <strelka/common/typedefs.hpp>

namespace strelka {
namespace trajectory {

inline float cubicBezier(float p0, float pf, float t) {
  return p0 + (t * t * t + 3 * (t * t * (1 - t))) * (pf - p0);
}

inline float cubicBezierDerivative(float p0, float pf, float t) {
  return (6 * t * (1 - t)) * (pf - p0);
}

inline float cubicBezierSecondDerivative(float p0, float pf, float t) {
  return (6 - 12 * t) * (pf - p0);
}

/**
 * @brief Remap t so that it's first phaseAmount% are mapped to the
 * phaseFraction% of the phase.
 *
 * We augment the swing speed using the below formula. For the first
 * phaseAmount% of the swing cycle, the swing leg moves faster and finishes
 * phaseFraction% of the full swing trajectory. The rest (1-phaseFraction)% of
 * trajectory takes another half swing cycle. Intuitely, we want to move the
 * swing foot quickly to the target landing location and stay above the ground,
 * in this way the control is more robust to perturbations to the body that may
 * cause the swing foot to drop onto the ground earlier than expected. This is
 * a common practice similar to the MIT cheetah and Marc Raibert's original
 * controllers.
 *
 * Erwin Coumans
 * @param t
 * @param phaseAmount
 * @param phaseFraction
 * @return float
 */
inline float speedupPhase(float t, float phaseAmount = 0.5,
                          float phaseFraction = 0.8) {
  if (t <= phaseAmount) {
    return phaseFraction * std::sin((t / phaseAmount) * (M_PI / 2));
  } else {
    return phaseFraction +
           (1 - phaseFraction) * (t - phaseAmount) / (1 - phaseAmount);
  }
}

/**
 * @brief Cubic Bezier trajectory generator
 *
 * @param order Bezier derivative order
 * @return Vec3<float> 3D point on a trajectory
 */
inline Vec3<float> getSwingTrajectory(const Vec3<float> &pStart,
                                      const Vec3<float> &pEnd,
                                      float swingHeight, float t,
                                      float swingDuration, bool swingBack,
                                      int order) {
  assert(order >= 0 && order < 3);

  float xyCoefficient, zCoefficient;
  float (*bezierFunc)(float, float, float);

  if (order == 0) {
    xyCoefficient = 1.0f;
    zCoefficient = 1.0f;
    bezierFunc = &cubicBezier;
  }

  if (order == 1) {
    xyCoefficient = swingDuration;
    zCoefficient = xyCoefficient / 2.0f;
    bezierFunc = &cubicBezierDerivative;
  }

  if (order == 2) {
    xyCoefficient = (swingDuration * swingDuration);
    zCoefficient = xyCoefficient / 4.0f;
    bezierFunc = &cubicBezierSecondDerivative;
  }

  Vec3<float> desiredPosition;
  float tSpedUp = speedupPhase(t);

  Vec3<float> swingMidpoint = pStart + (pEnd - pStart) / 2;

  if (swingBack) {
    Vec3<float> offset = (pEnd - pStart);
    offset(2) = 0;
    swingMidpoint = pStart - 0.04 * offset.normalized();
  }

  swingMidpoint(2) = pStart(2) + swingHeight;

  if (tSpedUp < 0.5) {
    desiredPosition(0) =
        bezierFunc(pStart(0), swingMidpoint(0), tSpedUp * 2) / xyCoefficient;
    desiredPosition(1) =
        bezierFunc(pStart(1), swingMidpoint(1), tSpedUp * 2) / xyCoefficient;
    desiredPosition(2) =
        bezierFunc(pStart(2), swingMidpoint(2), tSpedUp * 2) / zCoefficient;
  } else {
    desiredPosition(0) =
        bezierFunc(swingMidpoint(0), pEnd(0), tSpedUp * 2 - 1) / xyCoefficient;
    desiredPosition(1) =
        bezierFunc(swingMidpoint(1), pEnd(1), tSpedUp * 2 - 1) / xyCoefficient;
    desiredPosition(2) =
        bezierFunc(swingMidpoint(2), pEnd(2), tSpedUp * 2 - 1) / zCoefficient;
  }

  return desiredPosition;
}

} // namespace trajectory
} // namespace strelka

#endif // TRAJECTORY_H