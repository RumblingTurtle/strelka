
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
  // 6 for forces and 6 for moments
  static constexpr int ACTION_PER_LEG = 3;
  static constexpr int ACTION_DIM = NUM_LEGS * ACTION_PER_LEG;

  static constexpr float CONSTRAINT_MAX_SCALE = 10;
  static constexpr float CONSTRAINT_MIN_SCALE = 0.1;
  static constexpr float MPC_ALPHA_FORCE = 1e-5;
  static constexpr float FRICTION_COEFF = 0.45;
  // RPY , XYZ,  angular velocity, linear velocity
  static constexpr float MPC_WEIGHTS[STATE_DIM] = {
      1.0, 1.0, 0.0, 0.0, 0.0, 50.0, 0.0f, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0};

  using SolutionVectorType = Eigen::Matrix<float, ACTION_DIM, 1>;

  const float _bodyMass;
  const float timestep_;

  const int planning_horizon_;

  const Mat3<float> inertia_;
  const Mat3<float> inv_inertia_;

  bool initial_run_;

  // The following matrices will be updated for every call. However, their sizes
  // can be determined at class initialization time.
  FMat<float, STATE_DIM, STATE_DIM> _a_mat;
  FMat<float, STATE_DIM, STATE_DIM> _a_exp;
  FMat<float, STATE_DIM, ACTION_DIM> _b_mat;
  FMat<float, STATE_DIM + ACTION_DIM, STATE_DIM + ACTION_DIM> ab_concatenated_;

  // Contains all the power mats of _a_exp. Consider Eigen::SparseMatrix.
  DMat<float> _a_qp;   // STATE_DIM * horizon x STATE_DIM
  DMat<float> _b_qp;   // STATE_DIM * horizon x ACTION_DIM * horizon sparse
  DMat<float> _p_mat;  // ACTION_DIM * horizon x ACTION_DIM * horizon
  DVec<float> _q_vec;  // ACTION_DIM * horizon vector
  DVec<float> _u;      // ACTION_DIM * horizon vector
  DMat<float> _b_exps; // STATE_DIM * horizon x ACTION_DIM

  // Contains the constraint matrix and bounds.
  DMat<float> _constraint; // CONSTRAINT_DIM * NUM_LEGS  * horizon x ACTION_DIM
                           // * horizon
  DVec<float> _constraint_lb; // CONSTRAINT_DIM * NUM_LEGS * horizon
  DVec<float> _constraint_ub; // CONSTRAINT_DIM * NUM_LEGS * horizon

  DMat<float> b_qp_T_Q; // 2*b_qp*qp_weights
  FMat<float, STATE_DIM, STATE_DIM> qp_weights_block_;

  SolutionVectorType qp_solution_;
  ::OSQPWorkspace *workspace_ = nullptr;

  void updateConstraints(const DMat<bool> &contactTable);

  SolutionVectorType solveQP();

  void updateObjectiveVector(const DVec<float> &currentState,
                             const DMat<float> &bodyTrajectory);

  void
  computeABExponentials(const DVec<float> &currentState,
                        const DMat<float> &contactPositionsWorldFrameRotated,
                        const DMat<float> &bodyTrajectory);

  void computeQpMatrices();

  void fillQPWeights(const float *qp_weights);

  MPC(float mass, const Mat3<float> &inertia, int planning_horizon,
      float timestep);

  ~MPC();
  /**
   * @brief Compute reaction forces to
   *
   * @param currentState Robot's current state
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
  SolutionVectorType
  computeContactForces(const DVec<float> &currentState,
                       const DMat<bool> &contactTable,
                       const DMat<float> &contactPositionsWorldFrameRotated,
                       const DMat<float> &bodyTrajectory);

  void reset();
};
} // namespace control
} // namespace strelka

#endif // STRELKA_MPC_H