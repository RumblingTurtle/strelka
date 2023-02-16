#ifndef FOOTHOLD_PLANNER_H
#define FOOTHOLD_PLANNER_H

#include <common/A1/constants.hpp>
#include <common/constants.hpp>
#include <common/macros.hpp>
#include <common/rotation.hpp>
#include <common/typedefs.hpp>
#include <control/gait/GaitScheduler.hpp>
#include <messages/HighLevelCommand.hpp>
#include <robots/Robot.hpp>
#include <vector>

#define FOOT_CLEARANCE 0.02
#define FEEDBACK_GAIN 0.1

namespace strelka {

class FootholdPlanner {
  GaitScheduler &gaitScheduler;

  std::vector<std::vector<Vec3<float>>> footholds;

  float swingHeight[4];
  bool swingBack[4];

  Mat43<float> lastContactPosWorld;

  bool firstRun;
  bool updateContinuously;

public:
  enum class FOOTHOLD_PREDICTION_TYPE { SIMPLE, RAIBERT, CONTINUOUS };

  FootholdPlanner(GaitScheduler &gaitScheduler) : gaitScheduler(gaitScheduler) {
    footholds.resize(4);
    FOR_EACH_LEG { footholds[LEG_ID].reserve(5); }
  }

  void calculateFootholds(robots::Robot &robot,
                          messages::HighLevelCommand &command,
                          DMat<float> &bodyTrajectory,
                          DMat<bool> &contactTable) {

    Mat3<float> bodyToWorldRot = robot.bodyToWorldMat();
    FOR_EACH_LEG {

      if (firstRun) {
        footholds[LEG_ID][0] = robot.footPositionWorldFrame(LEG_ID);
      }

      bool updateAsStance = gaitScheduler.footInContact(LEG_ID) ||
                            gaitScheduler.lostContact(LEG_ID);

      if (updateAsStance) {
        lastContactPosWorld.block(LEG_ID, 0, 1, 3) =
            robot.footPositionWorldFrame(LEG_ID);
      }

      if (updateAsStance or gaitScheduler.swingStarted(LEG_ID) or
          updateContinuously) {

        footholds[LEG_ID].clear();
        footholds[LEG_ID].push_back(lastContactPosWorld.block(LEG_ID, 0, 1, 3));

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
            robot.positionWorldFrame(), bodyToWorldRot, footholds[LEG_ID][0],
            command.desiredLinearVelocityBodyFrame(),
            command.desiredAngularVelocityBodyFrame(),
            robot.linearVelocityBodyFrame(), LEG_ID, predictType);

        // Skip adjustment for now (no elevation map)
        Vec3<float> adjustedFootPosition;

        footholds[LEG_ID].push_back(predictedFootPosition);

        float heightDiff = footholds[LEG_ID][0](0) - footholds[LEG_ID][1](1);

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

    /*
            foot_table = np.zeros((4, len(body_plan)*3))
            foothold_idx = [0 for LEG_ID in range(4)]

            for h in range(len(body_plan))
                robot_R_h = rpy_to_R(body_plan[h][:3])
                robot_p_h = body_plan[h][3:6]

                for LEG_ID in range(4):
                    now_in_contact = contact_table[LEG_ID][h] == 1

                    first_horizon_step = h == 0

                    if not first_horizon_step:
                        was_in_swing = contact_table[LEG_ID][h-1] == 0
                        if was_in_swing and now_in_contact:
                            foothold_idx[LEG_ID] += 1

                            calculate_new_foothold = \
                                foothold_idx[LEG_ID] ==
       len(footholds[LEG_ID])

                            if calculate_new_foothold:
                                new_foothold = calculate_next_foot_pos(
                                    robot_p_h, robot_R_h,
        footholds[LEG_ID][-1], high_level_command.linearSpeed,
        high_level_command.angularVelocity[2], None, LEG_ID, "simple")
                                footholds[LEG_ID].append(new_foothold)

                    foot_table[LEG_ID][3*h:3*h+3] =
        footholds[LEG_ID][foothold_idx[LEG_ID]]-(
                        robot_p_h+robot_R_h@TRUNK_TO_COM_OFFSET)-np.array([0, 0,
        FOOT_RADIUS])

            return foot_table

        */

    if (firstRun) {
      firstRun = false;
    }
  }

  Vec3<float> predictNextFootPos(Vec3<float> currentPosition,
                                 Mat3<float> bodyToWorldRot,
                                 Vec3<float> prevFootPosition,
                                 Vec3<float> desiredLinearVelocity,
                                 Vec3<float> desiredAngularVelocity,
                                 Vec3<float> bodyLinearVelocity, int legId,
                                 FOOTHOLD_PREDICTION_TYPE predictType) {

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
          FEEDBACK_GAIN *
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
          footholds[LEG_ID][0],
          footholds[LEG_ID][1] + Vec3<float>{0, 0, FOOT_CLEARANCE},
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

} // namespace strelka

#endif // FOOTHOLD_PLANNER_H