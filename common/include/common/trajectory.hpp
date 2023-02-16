#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <common/typedefs.hpp>

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

inline float speedupPhase(float t, float phaseAmount = 0.5,
                          float phaseFraction = 0.8) {
  /*
    We augment the swing speed using the below formula. For the first
    phaseAmount% of the swing cycle, the swing leg moves faster and finishes
    phaseFraction% of the full swing trajectory. The rest (1-phaseFraction)% of
    trajectory takes another half swing cycle. Intuitely, we want to move the
    swing foot quickly to the target landing location and stay above the ground,
    in this way the control is more robust to perturbations to the body that may
    cause the swing foot to drop onto the ground earlier than expected. This is
    a common practice similar to the MIT cheetah and Marc Raibert's original
    controllers.
    */
  if (t <= phaseAmount) {
    return phaseFraction * std::sin((t / phaseAmount) * (M_PI / 2));
  } else {
    return phaseFraction +
           (1 - phaseFraction) * (t - phaseAmount) / (1 - phaseAmount);
  }
}

inline Vec3<float> getSwingTrajectoryPosition(Vec3<float> pStart,
                                              Vec3<float> pEnd,
                                              float swingHeight, float t,
                                              float swingDuration,
                                              bool swingBack) {
  Vec3<float> desiredPosition;
  float tSpedUp = speedupPhase(t);

  desiredPosition(0) = cubicBezier(pStart(0), pEnd(0), tSpedUp);
  desiredPosition(1) = cubicBezier(pStart(1), pEnd(1), tSpedUp);

  if (tSpedUp < 0.5) {
    desiredPosition(2) =
        cubicBezier(pStart(2), pEnd(2) + swingHeight, tSpedUp * 2);
  } else {
    desiredPosition(2) =
        cubicBezier(pStart(2) + swingHeight, pEnd(2), tSpedUp * 2 - 1);
  }

  return desiredPosition;
}

inline Vec3<float> getSwingTrajectoryVelocity(Vec3<float> pStart,
                                              Vec3<float> pEnd,
                                              float swingHeight, float t,
                                              float swingDuration,
                                              bool swingBack) {

  Vec3<float> desiredVelocity;
  float tSpedUp = speedupPhase(t);

  desiredVelocity(0) =
      cubicBezierDerivative(pStart(0), pEnd(0), tSpedUp) / swingDuration;
  desiredVelocity(1) =
      cubicBezierDerivative(pStart(1), pEnd(1), tSpedUp) / swingDuration;

  if (tSpedUp < 0.5) {
    desiredVelocity(2) =
        cubicBezierDerivative(pStart(2), pEnd(2) + swingHeight, tSpedUp * 2) *
        2 / swingDuration;
  } else {
    desiredVelocity(2) = cubicBezierDerivative(pStart(2) + swingHeight, pEnd(2),
                                               tSpedUp * 2 - 1) *
                         2 / swingDuration;
  }

  return desiredVelocity;
}

inline Vec3<float> getSwingTrajectoryAcceleration(Vec3<float> pStart,
                                                  Vec3<float> pEnd,
                                                  float swingHeight, float t,
                                                  float swingDuration,
                                                  bool swingBack) {
  Vec3<float> desiredAcceleration;
  float tSpedUp = speedupPhase(t);

  desiredAcceleration(0) =
      cubicBezierSecondDerivative(pStart(0), pEnd(0), tSpedUp) /
      (swingDuration * swingDuration);
  desiredAcceleration(1) =
      cubicBezierSecondDerivative(pStart(1), pEnd(1), tSpedUp) /
      (swingDuration * swingDuration);

  if (tSpedUp < 0.5) {
    desiredAcceleration(2) =
        cubicBezierSecondDerivative(pStart(2), pEnd(2) + swingHeight,
                                    tSpedUp * 2) *
        4 / (swingDuration * swingDuration);
  } else {
    desiredAcceleration(2) =
        cubicBezierSecondDerivative(pStart(2) + swingHeight, pEnd(2),
                                    tSpedUp * 2 - 1) *
        4 / (swingDuration * swingDuration);
  }

  return desiredAcceleration;
}

} // namespace trajectory
} // namespace strelka

#endif // TRAJECTORY_H