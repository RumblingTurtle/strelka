#include <control/FootholdPlanner.hpp>
namespace strelka {
namespace control {

FootholdPlanner::FootholdPlanner(GaitScheduler &gaitScheduler)
    : gaitScheduler(gaitScheduler), updateContinuously(false) {
  FOR_EACH_LEG { _footholds[LEG_ID].reserve(5); }
}

int FootholdPlanner::footholdCount(int legId) {
  assert(legId >= 0 && legId <= 3);
  return _footholds[legId].size();
}

Vec3<float> FootholdPlanner::getFoothold(int legId, int footholdId) {
  if (footholdId < footholdCount(legId)) {
    return _footholds[legId][footholdId];
  }
  throw InvalidFootholdIdx();
}

DMat<float> FootholdPlanner::calculateWorldFrameRotatedFootholds(
    robots::Robot &robot, messages::HighLevelCommand &command,
    DMat<float> &bodyTrajectory, DMat<bool> &contactTable) {
  const int horizonSteps = bodyTrajectory.rows();
  DMat<float> footholdTable(4, 3 * horizonSteps);
  footholdTable.setZero();
  int currentFootholdIdx[4] = {0, 0, 0, 0};

  for (int h = 0; h < horizonSteps; h++) {

    Vec3<float> bodyPosWorld = bodyTrajectory.block<1, 3>(h, 3).transpose();
    Mat3<float> bodyToWorldRot_h;
    rotation::rpy2rot(bodyTrajectory.block<1, 3>(h, 0), bodyToWorldRot_h);

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
                _footholds[LEG_ID][footholdCount(LEG_ID) - 1], robot, command,
                0.0, LEG_ID, FOOTHOLD_PREDICTION_TYPE::SIMPLE);

            _footholds[LEG_ID].push_back(newFoothold);
          }
        }
      }

      Vec3<float> footholdWorld =
          _footholds[LEG_ID][currentFootholdIdx[LEG_ID]] -
          Vec3<float>{0, 0, robot.footRadius()};

      footholdTable.block<1, 3>(LEG_ID, 3 * h) =
          (footholdWorld - bodyPosWorld).transpose();
    }
  }
  return footholdTable;
}

void FootholdPlanner::calculateNextFootholdPositions(
    robots::Robot &robot, messages::HighLevelCommand &command) {

  Mat3<float> bodyToWorldRot = robot.bodyToWorldMat();
  FOR_EACH_LEG {
    bool updateAsStance = gaitScheduler.footInContact(LEG_ID) ||
                          gaitScheduler.lostContact(LEG_ID);

    bool swingStarted = gaitScheduler.swingStarted(LEG_ID);

    if (updateAsStance || firstRun) {
      lastContactPosWorld.block<1, 3>(LEG_ID, 0) =
          robot.footPositionWorldFrame(LEG_ID).transpose();
    }

    if (updateAsStance || swingStarted || updateContinuously) {
      _footholds[LEG_ID].clear();
      _footholds[LEG_ID].push_back(
          lastContactPosWorld.block<1, 3>(LEG_ID, 0).transpose());

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
          robot, command, FEEDBACK_GAIN, LEG_ID, predictType);

      // Skip adjustment for now (no elevation map)
      // Vec3<float> adjustedFootPosition;

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

      swingHeight[LEG_ID] = command.desiredFootHeight();
      swingBack[LEG_ID] = false;
    }
  }

  if (firstRun) {
    firstRun = false;
  }
}

Vec3<float> FootholdPlanner::predictNextFootPos(
    Vec3<float> currentPosition, Mat3<float> bodyToWorldRot,
    Vec3<float> prevFootPosition, robots::Robot &robot,
    messages::HighLevelCommand &command, float feedbackGain, int legId,
    FOOTHOLD_PREDICTION_TYPE predictType) {
  Vec3<float> trunkToThighOffset = robot.trunkToThighOffset(legId);

  Vec3<float> linearizedAngularVelocityWorld =
      bodyToWorldRot *
      command.desiredAngularVelocityBodyFrame().cross(trunkToThighOffset);

  Vec3<float> desiredVelocityWorld =
      linearizedAngularVelocityWorld +
      bodyToWorldRot * command.desiredLinearVelocityBodyFrame();

  // TODO implement hip offset
  Vec3<float> hipPosWorld =
      bodyToWorldRot * trunkToThighOffset + currentPosition;

  Vec3<float> predictedFootWorld;
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
        feedbackGain * (bodyToWorldRot * robot.linearVelocityBodyFrame() -
                        desiredVelocityWorld);
  }

  /**
   * Assume the next foothold's height is on the same level as the starting
   * position.
   */
  predictedFootWorld(2) = robot.footRadius() + _footholds[legId].at(0)(2);
  return predictedFootWorld;
}

void FootholdPlanner::getFootDesiredPVA(
    robots::Robot &robot, Vec12<float> &desiredFootPositions,
    Vec12<float> &desiredFootVelocities,
    Vec12<float> &desiredFootAccelerations) {

  FOR_EACH_LEG {
    Vec3<float> desiredFootPosition;
    Vec3<float> desiredFootVelocity;
    Vec3<float> desiredFootAcceleration;

    if (gaitScheduler.isLegSwinging(LEG_ID)) {

      Vec3<float> pStart = _footholds[LEG_ID][0];
      Vec3<float> pEnd = _footholds[LEG_ID][1];

      desiredFootPosition = trajectory::getSwingTrajectoryPosition(
          pStart, pEnd, swingHeight[LEG_ID],
          gaitScheduler.normalizedPhase(LEG_ID),
          gaitScheduler.swingDuration(LEG_ID), swingBack[LEG_ID]);

      desiredFootVelocity = trajectory::getSwingTrajectoryVelocity(
          pStart, pEnd, swingHeight[LEG_ID],
          gaitScheduler.normalizedPhase(LEG_ID),
          gaitScheduler.swingDuration(LEG_ID), swingBack[LEG_ID]);

      desiredFootAcceleration = trajectory::getSwingTrajectoryAcceleration(
          pStart, pEnd, swingHeight[LEG_ID],
          gaitScheduler.normalizedPhase(LEG_ID),
          gaitScheduler.swingDuration(LEG_ID), swingBack[LEG_ID]);

    } else {
      desiredFootPosition = robot.footPositionWorldFrame(LEG_ID);
      desiredFootVelocity = {0, 0, 0};
      desiredFootAcceleration = {0, 0, 0};
    }

    desiredFootPositions.block<3, 1>(LEG_ID * 3, 0) = desiredFootPosition;
    desiredFootVelocities.block<3, 1>(LEG_ID * 3, 0) = desiredFootVelocity;
    desiredFootAccelerations.block<3, 1>(LEG_ID * 3, 0) =
        desiredFootAcceleration;
  }
}

} // namespace control
} // namespace strelka
