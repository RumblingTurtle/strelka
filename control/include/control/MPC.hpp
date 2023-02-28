
#ifndef STRELKA_MPC_H
#define STRELKA_MPC_H
#include <common/constants.hpp>
#include <common/rotation.hpp>
#include <common/typedefs.hpp>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Sparse>
#include <eigen3/unsupported/Eigen/src/MatrixFunctions/MatrixExponential.h>
#include <osqp/ctrlc.h>
#include <osqp/osqp.h>
#include <robots/Robot.hpp>

namespace strelka {
class MPC {
  const double _bodyMass;
  const Mat3<double> inertia_;
  const Mat3<double> inv_inertia_;
  const int planning_horizon_;
  const double timestep_;

  // 13 * horizon diagonal matrix.
  SMat<double> qp_weights_;

  // NUM_LEGS * 3 * horizon diagonal matrix.
  const double alpha_;

  // The following matrices will be updated for every call. However, their sizes
  // can be determined at class initialization time.
  DMat<double> _a_mat;           // 13 x 13
  DMat<double> _b_mat;           // 13 x (NUM_LEGS * 3)
  DMat<double> ab_concatenated_; // 13 + NUM_LEGS * 3 x 13 + NUM_LEGS * 3
  DMat<double> _a_exp;           // same dimension as _a_mat
  DMat<double> _b_exp;           // same dimension as _b_mat

  // Contains all the power mats of _a_exp. Consider Eigen::SparseMatrix.
  DMat<double> _a_qp;  // 13 * horizon x 13
  DMat<double> _b_qp;  // 13 * horizon x NUM_LEGS * 3 * horizon sparse
  DMat<double> _p_mat; // NUM_LEGS * 3 * horizon x NUM_LEGS * 3 * horizon
  DVec<double> _q_vec; // NUM_LEGS * 3 * horizon vector

  DMat<double> _b_exps; // 13 * horizon x (NUM_LEGS * 3 )

  // Contains the constraint matrix and bounds.
  DMat<double> _constraint; // 5 * NUM_LEGS * horizon x 3 * NUM_LEGS * horizon
  DVec<double> _constraint_lb; // 5 * NUM_LEGS * horizon
  DVec<double> _constraint_ub; // 5 * NUM_LEGS * horizon

  Vec4<double> _footFrictionCoefficients;
  double kMaxScale;
  double kMinScale;

  DVec<double> qp_solution_;
  ::OSQPWorkspace *workspace_;
  // Whether optimizing for the first step
  bool initial_run_;

  void updateConstraints(DMat<bool> &contactTable);

  DVec<double> &solveQP();

  void updateObjectiveVector(robots::Robot &robot, DMat<float> &bodyTrajectory);

  void computeABExponentials(robots::Robot &robot,
                             DMat<float> &contactPositionsWorldFrameRotated,
                             DMat<float> &bodyTrajectory);

  void computeQpMatrices();

public:
  // 6 dof pose + 6 dof velocity + 1 gravity
  static constexpr int STATE_DIM = 13;
  static constexpr int CONSTRAINT_DIM = 5;
  static constexpr int NUM_LEGS = 4;
  static constexpr int ACTION_DIM = NUM_LEGS * 3;

  void fillQPWeights(const DVec<double> &qp_weights);
  MPC(double mass, const Vec3<double> &inertia, int planning_horizon,
      double timestep, const DVec<double> &qp_weights, double alpha,
      const Vec4<double> &_footFrictionCoefficients, double kMaxScale,
      double kMinScale);

  ~MPC();

  DVec<double> &
  computeContactForces(robots::Robot &robot, DMat<bool> &contactTable,
                       DMat<float> &contactPositionsWorldFrameRotated,
                       DMat<float> &bodyTrajectory);

  void reset();
};

} // namespace strelka

#endif // STRELKA_MPC_H