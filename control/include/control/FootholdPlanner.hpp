#ifndef FOOTHOLD_PLANNER_H
#define FOOTHOLD_PLANNER_H

#include <common/A1/constants.hpp>
#include <common/constants.hpp>
#include <common/macros.hpp>
#include <common/rotation.hpp>
#include <common/typedefs.hpp>
#include <control/gait/GaitScheduler.hpp>
#include <exception>
#include <messages/HighLevelCommand.hpp>
#include <robots/Robot.hpp>
#include <vector>

#define FOOT_CLEARANCE 0.02
#define FEEDBACK_GAIN 0.1

namespace strelka {
namespace control {

class InvalidFootholdIdx {
  const char *what() {
    return "FootholdPlanner: invalid foothold index in getFoothold";
  }
};

class FootholdPlanner {
  GaitScheduler &gaitScheduler;

  std::vector<std::vector<Vec3<float>>> _footholds;

  float swingHeight[4];
  bool swingBack[4];

  Mat43<float> lastContactPosWorld;

  bool firstRun;
  bool updateContinuously;

public:
  enum class FOOTHOLD_PREDICTION_TYPE { SIMPLE, RAIBERT, CONTINUOUS };

  FootholdPlanner(GaitScheduler &gaitScheduler) : gaitScheduler(gaitScheduler) {
    _footholds.resize(4);
    FOR_EACH_LEG { _footholds[LEG_ID].reserve(5); }
  }

  int footholdCount(int legId) {
    assert(legId >= 0 && legId <= 3);
    return _footholds[legId].size();
  }

  Vec3<float> getFoothold(int legId, int footholdId) {
    if (footholdId < footholdCount(legId)) {
      return _footholds[legId][footholdId];
    }
    throw InvalidFootholdIdx();
  }

  DMat<float> calculateBodyFrameFootholds(robots::Robot &robot,
                                          messages::HighLevelCommand &command,
                                          DMat<float> &bodyTrajectory,
                                          DMat<bool> &contactTable) {

    int horizonSteps = bodyTrajectory.rows();
    DMat<float> footholdTable(4, 3 * horizonSteps);

    int currentFootholdIdx[4] = {0, 0, 0, 0};

    for (int h = 0; h < horizonSteps; h++) {
      Mat3<float> bodyToWorldRot_h;

      Vec3<float> bodyPosWorld;
      rotation::rpy2rot(bodyTrajectory.block(h, 0, 1, 3).transpose(),
                        bodyToWorldRot_h);
      bodyPosWorld = bodyTrajectory.block(h, 3, 1, 3).transpose();

      Mat3<float> bodyToWorldRotInv_h = bodyToWorldRot_h.transpose();

      FOR_EACH_LEG {
        bool inStance = contactTable(LEG_ID, h);
        if (h != 0) {
          bool wasInSwing = !contactTable(LEG_ID, h - 1);
          if (wasInSwing && inStance) {
            currentFootholdIdx[LEG_ID] += 1;
            bool calculateNewFoothold =
                footholdCount(LEG_ID) == currentFootholdIdx[LEG_ID];

            if (calculateNewFoothold) {
              // Setting linear desired and actual velocities to the same value
              // as a hackish way to remove feedback term
              Vec3<float> newFoothold = predictNextFootPos(
                  bodyPosWorld, bodyToWorldRot_h,
                  _footholds[LEG_ID][footholdCount(LEG_ID) - 1],
                  command.desiredLinearVelocityBodyFrame(),
                  command.desiredAngularVelocityBodyFrame(),
                  command.desiredLinearVelocityBodyFrame(), 0.0, LEG_ID,
                  FOOTHOLD_PREDICTION_TYPE::SIMPLE);

              _footholds[LEG_ID].push_back(newFoothold);
            }
          }
        }

        Vec3<float> footholdWorld =
            _footholds[LEG_ID][currentFootholdIdx[LEG_ID]] -
            Vec3<float>{0, 0, constants::A1::FOOT_RADIUS};

        Vec3<float> footholdBody_h =
            bodyToWorldRotInv_h * (footholdWorld - bodyPosWorld);

        footholdTable.block(LEG_ID, 3 * h, 1, 3) = footholdBody_h.transpose();
      }
    }
    return footholdTable;
  }

  void calculateNextFootholdPositions(robots::Robot &robot,
                                      messages::HighLevelCommand &command) {

    Mat3<float> bodyToWorldRot = robot.bodyToWorldMat();
    FOR_EACH_LEG {

      if (firstRun) {
        _footholds[LEG_ID][0] = robot.footPositionWorldFrame(LEG_ID);
      }

      bool updateAsStance = gaitScheduler.footInContact(LEG_ID) ||
                            gaitScheduler.lostContact(LEG_ID);

      if (updateAsStance) {
        lastContactPosWorld.block(LEG_ID, 0, 1, 3) =
            robot.footPositionWorldFrame(LEG_ID).transpose();
      }

      if (updateAsStance or gaitScheduler.swingStarted(LEG_ID) or
          updateContinuously) {

        _footholds[LEG_ID].clear();
        _footholds[LEG_ID].push_back(
            lastContactPosWorld.block(LEG_ID, 0, 1, 3).transpose());

        FOOTHOLD_PREDICTION_TYPE predictType;
        if (updateContinuously) {
          if (updateAsStance) {
            predictType = FOOTHOLD_PREDICTION_TYPE::SIMPLE;
          } else {
            predictType = FOOTHOLD_PREDICTION_TYPE::CONTINUOUS;
          }
        } else {
          if (updateAsStance) {
            predictType = FOOTHOLD_PREDICTION_TYPE::SIMPLE;
          } else {
            predictType = FOOTHOLD_PREDICTION_TYPE::RAIBERT;
          }
        }

        Vec3<float> predictedFootPosition = predictNextFootPos(
            robot.positionWorldFrame(), bodyToWorldRot, _footholds[LEG_ID][0],
            command.desiredLinearVelocityBodyFrame(),
            command.desiredAngularVelocityBodyFrame(),
            robot.linearVelocityBodyFrame(), FEEDBACK_GAIN, LEG_ID,
            predictType);

        // Skip adjustment for now (no elevation map)
        Vec3<float> adjustedFootPosition;

        _footholds[LEG_ID].push_back(predictedFootPosition);

        float heightDiff = _footholds[LEG_ID][0](0) - _footholds[LEG_ID][1](1);

        bool steppingUp = heightDiff > command.desiredFootHeight();
        bool steppingDown = heightDiff < -command.desiredFootHeight() / 4;

        if (steppingUp) {
          swingHeight[LEG_ID] = heightDiff + 0.02;
          swingBack[LEG_ID] = true;
        }

        if (steppingDown) {
          swingHeight[LEG_ID] = 0.02;
          swingBack[LEG_ID] = false;
        }

        if (!steppingUp && !steppingDown) {
          swingHeight[LEG_ID] = command.desiredFootHeight();
          swingBack[LEG_ID] = false;
        }
      }
    }

    if (firstRun) {
      firstRun = false;
    }
  }

  Vec3<float> predictNextFootPos(
      Vec3<float> currentPosition, Mat3<float> bodyToWorldRot,
      Vec3<float> prevFootPosition, Vec3<float> desiredLinearVelocity,
      Vec3<float> desiredAngularVelocity, Vec3<float> bodyLinearVelocity,
      float feedbackGain, int legId, FOOTHOLD_PREDICTION_TYPE predictType) {

    Vec3<float> predictedFootWorld;

    Vec3<float> trunkToThighOffset = Eigen::Map<const Vec3<float>>(
        constants::A1::TRUNK_TO_THIGH_OFFSETS + legId * 3, 3);

    Vec3<float> linearizedAngularVelocityWorld =
        bodyToWorldRot * desiredAngularVelocity.cross(trunkToThighOffset);

    Vec3<float> desiredVelocityWorld =
        linearizedAngularVelocityWorld + bodyToWorldRot * desiredLinearVelocity;

    // TOODO implement hip offset
    Vec3<float> hipPosWorld =
        bodyToWorldRot * trunkToThighOffset + currentPosition;

    switch (predictType) {
    case FOOTHOLD_PREDICTION_TYPE::CONTINUOUS:
      predictedFootWorld =
          hipPosWorld +
          (1 - gaitScheduler.normalizedPhase(legId)) *
              gaitScheduler.swingDuration(legId) * desiredVelocityWorld +
          0.5 * gaitScheduler.stanceDuration(legId) * desiredVelocityWorld;
      break;
    case FOOTHOLD_PREDICTION_TYPE::RAIBERT:
      predictedFootWorld =
          hipPosWorld +
          gaitScheduler.swingDuration(legId) * desiredVelocityWorld +
          0.5 * gaitScheduler.stanceDuration(legId) * desiredVelocityWorld;
      break;
    case FOOTHOLD_PREDICTION_TYPE::SIMPLE:
      predictedFootWorld =
          prevFootPosition +
          desiredVelocityWorld * gaitScheduler.phaseDuration(legId);
      break;
    }

    if (predictType != FOOTHOLD_PREDICTION_TYPE::SIMPLE) {
      predictedFootWorld +=
          feedbackGain *
          (bodyToWorldRot * bodyLinearVelocity - desiredVelocityWorld);
    }
    return predictedFootWorld;
  }

  void getFootDesiredPVA(Vec12<float> &footP, Vec12<float> &footV,
                         Vec12<float> &footA) {
    FOR_EACH_LEG {
      /*
    if (gaitScheduler.isLegScheduledToSwing(LEG_ID)) {
      foot_positions[LEG_ID], foot_velocities[LEG_ID],
          foot_accelerations[LEG_ID];

      getSwingTrajectory(
          _footholds[LEG_ID][0],
          _footholds[LEG_ID][1] + Vec3<float>{0, 0, FOOT_CLEARANCE},
          swingHeight[LEG_ID], gaitScheduler.normalizedPhase(LEG_ID),
          gaitScheduler.swingDuration(LEG_ID), swingBack[LEG_ID]);

    } else {
      foot_positions[LEG_ID] = currentFootPosWorld.row(LEG_ID);
      foot_velocities[LEG_ID] = {0, 0, 0};
      foot_accelerations[LEG_ID] = {0, 0, 0};
    }
  }
    */
    }
  }
};
} // namespace control
} // namespace strelka

#endif // FOOTHOLD_PLANNER_H