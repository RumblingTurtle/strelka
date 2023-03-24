# Defining your own robot


## Define your robot

In order to use all of the strelka modules packages you have to inherit and provide implementations of a Robot interface class located at [strelka/include/strelka/robots/Robot.hpp](strelka/include/strelka/robots/Robot.hpp) and put it under the same source and header directory. Run CMake again so that the new source files are discovered properly.

### Leg order notation
```
 legId     name        abb.
   0    Front right    (FR)
   1    Front left     (FL)
   2    Rear right     (RR)
   3    Rear left      (RL)
```

*World and odometry frame are interexchangable. All coordinate frames are right handed.

## 1. Define constant values for [Robot.hpp](strelka/include/strelka/robots/Robot.hpp)
```c++
  /**
   * Offset from the center of the robot body to it's thigh and hip links
   * Needed for estimation of the next nominal position when planning footholds
   */
  Vec3<float> trunkToThighOffset(int legId);
  Vec3<float> trunkToHipOffset(int legId);

  /**
   * Radius of the foot link.
   *
   * Since we are planning movement of the foot center \\
   * it is important to know it's radius \\
   * in order to avoid collisions
   */
  const float footRadius();

  // Center of mass offset of the robot for WBIC and MPC
  Vec3<float> bodyToComOffset();

  // X Y Z dimensions of the body link
  Vec3<float> bodyDimensions();

  // Link lengths in the order: Thigh hip knee
  Vec3<float> legDimensions();

  // Trunk link mass for WBIC
  const float trunkMass();

  // Mass of the whole robot for MPC
  const float robotMass();

  // Inertia matrix
  Mat3<float> rotationalInertia();

  // Angle configuration of the foot for initialization state. Usually 
  // represented by the angles during encoder calibration stage.
  Vec3<float> initAngles();

  // Angle configuration for standing
  Vec3<float> standAngles();

  // Damping and position gains for servos
  // Note that the same values are used for every triplet 
  // representing each leg
  Vec3<float> positionGains();
  Vec3<float> dampingGains();

```

## 2. Define required implementations for [Robot.hpp](strelka/include/strelka/robots/Robot.hpp)
```c++
  // Check if the foot position in the world frame is 
  // kinematically reachable.
  bool worldFrameIKCheck(Vec3<float> footPositionWorldFrame,
                                 int legId);

  // Joint to end effector velocity mapping matrices
  Mat3<float> footJacobianImpl(int legId);

  // Trunk frame foot position
  Vec3<float> footPositionTrunkFrameImpl(int legId);

  // Contact estimate calculation
  bool estimateContactImpl(int legId);
```
## 3. Add your header to [Robots.hpp](strelka/include/strelka/robots/Robots.hpp)

## 4. And template specifications of your robot to [RobotRegistry.hpp](strelka/include/strelka/robots/RobotRegistry.hpp)

```c++
#ifdef STATE_ESTIMATOR_NODE_HEADER
template class state_estimation::StateEstimatorNode<robots::UnitreeA1>;
template class state_estimation::StateEstimatorNode<robots::MYROBOT>; // <-- Here
#undef STATE_ESTIMATOR_NODE_HEADER
#endif

#ifdef WBIC_NODE_HEADER
template class control::WBICNode<robots::UnitreeA1>;
template class control::WBICNode<robots::MYROBOT>; // <-- Here
#undef WBIC_NODE_HEADER
#endif

#ifdef LOCAL_PLANNER_NODE_HEADER
template class control::LocalPlannerNode<robots::UnitreeA1>;
template class control::LocalPlannerNode<robots::MYROBOT>; // <-- Here
#undef LOCAL_PLANNER_NODE_HEADER
#endif

#ifdef MOVE_TO_INTERFACE_HEADER
template class interfaces::MoveToInterface<robots::UnitreeA1>;
template class interfaces::MoveToInterface<robots::MYROBOT>; // <-- Here
#undef MOVE_TO_INTERFACE_HEADER
#endif
```

## 4. Write lcm publishers for state and command messages
[robot_raw_state.lcm](strelka_messages/lcm/robot_raw_state.lcm) -> strelka_lcm_headers/RobotRawState.hpp 
```c
struct RobotRawState
{
    // IMU
    float    quaternion[4];
    float    gyro[3];
    float    accel[3];
    // Foot force sensors
    float    footForces[4];
    // Motor states
    float    q[12];
    float    dq[12];
    int8_t   temperature[12];
    float    tau[12];
    // Robot timer
    float  tick;

    // Robot's own odometry. Not required!
    float    position[3];
    float    velocity[3];
    float    angularVelocity[3];
    // High level flag. Not required!
    int8_t   levelFlag;
}
```

[robot_low_command.lcm](strelka_messages/lcm/robot_low_command.lcm) -> strelka_lcm_headers/RobotLowCommand.hpp 
```c
// 12 angles, velocities, gains and torques
struct RobotLowCommand
{
    float   q[12];
    float   dq[12];
    float   kp[12];
    float   kd[12];
    float   tau[12];
}
```

## Real example: [UnitreeA1](strelka/include/strelka/robots/A1/UnitreeA1.hpp)
[strelka_ros/include/strelka_ros/A1/A1Bridge.hpp](https://github.com/RumblingTurtle/strelka_ros/blob/master/strelka_ros/include/strelka_ros/A1/A1Bridge.hpp) provides an example for command and state publishers in a simulation setting