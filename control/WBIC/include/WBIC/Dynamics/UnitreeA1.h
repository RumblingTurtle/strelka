#ifndef UNITREE_A1_H
#define UNITREE_A1_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Unitree A1
 */
template <typename T> Quadruped<T> buildA1() {
  Quadruped<T> a1;
  a1._robotType = RobotType::A1;

  a1._bodyMass = 6.0;
  a1._bodyLength = 0.1805 * 2;
  a1._bodyWidth = 0.047 * 2;
  a1._bodyHeight = 0.01675 * 2;
  a1._abadGearRatio = 9;
  a1._hipGearRatio = 9;
  a1._kneeGearRatio = 9;
  a1._abadLinkLength = 0.0838;
  a1._hipLinkLength = 0.2;
  // a1._kneeLinkLength = 0.175;
  // a1._maxLegLength = 0.384;
  a1._kneeLinkY_offset = 0.0;
  // a1._kneeLinkLength = 0.20;
  a1._kneeLinkLength = 0.2;
  a1._maxLegLength = 0.4;

  // Motor parameters are not used! Not changed
  a1._motorTauMax = 3.f;
  a1._batteryV = 24;
  a1._motorKT = .05; // this is flux linkage * pole pairs
  a1._motorR = 0.173;
  a1._jointDamping = .01;
  a1._jointDryFriction = .2;
  // a1._jointDamping = .0;
  // a1._jointDryFriction = .0;

  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 0, 0, 0, 0, 0, 0, 0, 0, 0; // ZERO ROTOR INERTIA
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 402, 0, 0, 0, 691, 0, 0, 0, 487;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(-0.003875, 0.001622, 0.000042); // LEFT
  SpatialInertia<T> abadInertia(0.595, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 5251, 0, 0, 0, 5000, 0, 0, 0, 1110;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(-0.003574, -0.019529, -0.030323);
  SpatialInertia<T> hipInertia(0.888, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 2344, 0, 0, 0, 2360, 0, 0, 0, 31;
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(0.007105, -0.000239, -0.096933);
  SpatialInertia<T> kneeInertia(0.151, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.0, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.0, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 15853, 0, 0, 0, 37799, 0, 0, 0, 45654;
  bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0, 0.0041, -0.0005);
  SpatialInertia<T> bodyInertia(a1._bodyMass, bodyCOM, bodyRotationalInertia);

  a1._abadInertia = abadInertia;
  a1._hipInertia = hipInertia;
  a1._kneeInertia = kneeInertia;
  a1._abadRotorInertia = rotorInertiaX;
  a1._hipRotorInertia = rotorInertiaY;
  a1._kneeRotorInertia = rotorInertiaY;
  a1._bodyInertia = bodyInertia;

  // locations
  a1._abadRotorLocation = Vec3<T>(0, 0, 0);
  a1._abadLocation = Vec3<T>(a1._bodyLength, a1._bodyWidth, 0) * 0.5;
  a1._hipLocation = Vec3<T>(0, a1._abadLinkLength, 0);
  a1._hipRotorLocation = Vec3<T>(0, 0, 0);
  a1._kneeLocation = Vec3<T>(0, 0, -a1._hipLinkLength);
  a1._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return a1;
}

#endif // PROJECT_A1_H
