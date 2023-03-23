
#ifndef STRELKA_MPC_H
#define STRELKA_MPC_H
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Sparse>
#include <eigen3/unsupported/Eigen/src/MatrixFunctions/MatrixExponential.h>
#include <strelka/common/constants.hpp>
#include <strelka/common/rotation.hpp>
#include <strelka/common/typedefs.hpp>
#include <strelka/robots/Robot.hpp>

struct OSQPWorkspace;

namespace strelka {
namespace control {
/**
 * @brief Model predictive stance controller
 * https://ieeexplore.ieee.org/document/8594448/
 *
 */
class MPC {
public:
  // 6 dof pose + 6 dof velocity + 1 gravity
  static constexpr int STATE_DIM = 13;
  static constexpr int CONSTRAINT_DIM = 5;
  static constexpr int NUM_LEGS = 4;
  static constexpr int ACTION_DIM = NUM_LEGS * 3;

  static constexpr float CONSTRAINT_MAX_SCALE = 10;
  static constexpr float CONSTRAINT_MIN_SCALE = 0.1;
  static constexpr float MPC_ALPHA = 1e-5;
  static constexpr float FRICTION_COEFFS[4] = {0.45, 0.45, 0.45, 0.45};
  static constexpr float MPC_WEIGHTS[13] = {1.0, 1.0, 0.0, 0.0, 0.0, 50.0, 0.0f,
                                            0.0, 1.0, 1.0, 1.0, 0.0, 0.0};

private:
  const float _bodyMass;
  const float alpha_;
  const float timestep_;

  float kMaxScale;
  float kMinScale;

  const int planning_horizon_;

  const Mat3<float> inertia_;
  const Mat3<float> inv_inertia_;
  Vec4<float> _footFrictionCoefficients;

  bool initial_run_;

  // The following matrices will be updated for every call. However, their sizes
  // can be determined at class initialization time.
  FMat<float, STATE_DIM, STATE_DIM> _a_mat;
  FMat<float, STATE_DIM, STATE_DIM> _a_exp;
  FMat<float, STATE_DIM, ACTION_DIM> _b_mat;
  FMat<float, STATE_DIM, ACTION_DIM> _b_exp;
  FMat<float, STATE_DIM + ACTION_DIM, STATE_DIM + ACTION_DIM> ab_concatenated_;

  // Contains all the power mats of _a_exp. Consider Eigen::SparseMatrix.
  DMat<float> _a_qp;  // 13 * horizon x 13
  DMat<float> _b_qp;  // 13 * horizon x NUM_LEGS * 3 * horizon sparse
  DMat<float> _p_mat; // NUM_LEGS * 3 * horizon x NUM_LEGS * 3 * horizon
  DVec<float> _q_vec; // NUM_LEGS * 3 * horizon vector

  DMat<float> _b_exps; // 13 * horizon x (NUM_LEGS * 3)

  // Contains the constraint matrix and bounds.
  DMat<float> _constraint;    // 5 * NUM_LEGS * horizon x 3 * NUM_LEGS * horizon
  DVec<float> _constraint_lb; // 5 * NUM_LEGS * horizon
  DVec<float> _constraint_ub; // 5 * NUM_LEGS * horizon

  SMat<float> qp_weights_;
  DVec<float> qp_solution_;

  ::OSQPWorkspace *workspace_;
  // Whether optimizing for the first step

  void updateConstraints(const DMat<bool> &contactTable);

  DVec<float> &solveQP();

  void updateObjectiveVector(robots::Robot &robot,
                             const DMat<float> &bodyTrajectory);

  void
  computeABExponentials(robots::Robot &robot,
                        const DMat<float> &contactPositionsWorldFrameRotated,
                        const DMat<float> &bodyTrajectory);

  void computeQpMatrices();

  void fillQPWeights(const float *qp_weights);

public:
  MPC(float mass, const Mat3<float> &inertia, int planning_horizon,
      float timestep);

  ~MPC();
  /**
   * @brief Compute reaction forces to
   *
   * @param robot Object which implements Robot interface
   *
   * @param contactTable Table of size 4xN with contact/no contact pattern per
   * each leg. See GaitScheduler's getContactTable for details
   *
   * @param contactPositionsWorldFrameRotated Table of size 4x3*N with each
   * triplet in a row representing foothold position in world frame at step n
   * minus body position at that step
   *
   * @param bodyTrajectory Table of size Nx13 which contains body trajectory.
   * See BodyTrajectoryPlanner for an example
   *
   */
  DVec<float> &
  computeContactForces(robots::Robot &robot, const DMat<bool> &contactTable,
                       const DMat<float> &contactPositionsWorldFrameRotated,
                       const DMat<float> &bodyTrajectory);

  void reset();
};
} // namespace control
} // namespace strelka

#endif // STRELKA_MPC_H