#include <strelka/control/MPC.hpp>

#include <iostream>
#include <osqp/ctrlc.h>
#include <osqp/osqp.h>

namespace strelka {
namespace control {

constexpr int MPC::STATE_DIM;
constexpr int MPC::CONSTRAINT_DIM;
constexpr int MPC::NUM_LEGS;
constexpr int MPC::ACTION_DIM;

constexpr float MPC::CONSTRAINT_MAX_SCALE;
constexpr float MPC::CONSTRAINT_MIN_SCALE;
constexpr float MPC::FRICTION_COEFF;
constexpr float MPC::MPC_WEIGHTS[MPC::STATE_DIM];

MPC::MPC(float mass, const Mat3<float> &inertia, int planning_horizon,
         float timestep)
    : _bodyMass(mass), inertia_(inertia), inv_inertia_(inertia_.inverse()),
      planning_horizon_(planning_horizon), timestep_(timestep),
      b_qp_T_Q(ACTION_DIM * planning_horizon, STATE_DIM * planning_horizon),
      _a_qp(STATE_DIM * planning_horizon, STATE_DIM),
      _b_qp(STATE_DIM * planning_horizon, ACTION_DIM * planning_horizon),
      _p_mat(ACTION_DIM * planning_horizon, ACTION_DIM * planning_horizon),
      _q_vec(ACTION_DIM * planning_horizon),
      _constraint(CONSTRAINT_DIM * NUM_LEGS * planning_horizon,
                  ACTION_DIM * planning_horizon),
      _constraint_lb(CONSTRAINT_DIM * NUM_LEGS * planning_horizon),
      _constraint_ub(CONSTRAINT_DIM * NUM_LEGS * planning_horizon),
      workspace_(0), initial_run_(true), _u(ACTION_DIM * planning_horizon),
      _b_exps(planning_horizon * STATE_DIM, ACTION_DIM) {

  qp_weights_block_.setZero();
  fillQPWeights(MPC_WEIGHTS);
  qp_solution_.setZero();
  b_qp_T_Q.setZero();
  _a_mat.setZero();
  _b_mat.setZero();
  ab_concatenated_.setZero();
  _a_exp.setZero();
  _a_qp.setZero();
  _b_qp.setZero();
  _constraint.setZero();
  _constraint_lb.setZero();
  _constraint_ub.setZero();
  _b_exps.setZero();
  _p_mat.setZero();
}

void MPC::fillQPWeights(const float *qp_weights) {
  for (int i = 0; i < MPC::STATE_DIM; i++) {
    qp_weights_block_(i, i) = qp_weights[i];
  }
}

MPC::~MPC() { osqp_cleanup(workspace_); }

void MPC::computeABExponentials(
    const DVec<float> &currentState,
    const DMat<float> &contactPositionsWorldFrameRotated,
    const DMat<float> &bodyTrajectory) {
  // A mat calculation
  float avg_yaw = bodyTrajectory.block(0, 2, planning_horizon_, 1).mean();
  // The transformation of angular velocity to roll pitch yaw rate. Caveat:
  // rpy rate is not a proper vector and does not follow the common vector
  // transformation dicted by the rotation matrix. Here we assume the input
  // rotation is in X->Y->Z order in the extrinsic/fixed frame, or z->y'->x''
  // order in the intrinsic frame.
  const float cos_yaw = cos(avg_yaw);
  const float sin_yaw = sin(avg_yaw);
  const float pitch = currentState(1); // Pitch
  const float cos_pitch = cos(pitch);
  const float tan_pitch = tan(pitch);
  Mat3<float> angular_velocity_to_rpy_rate;
  angular_velocity_to_rpy_rate << cos_yaw / cos_pitch, sin_yaw / cos_pitch, 0,
      -sin_yaw, cos_yaw, 0, cos_yaw * tan_pitch, sin_yaw * tan_pitch, 1;

  _a_mat.block<3, 3>(0, 6) = angular_velocity_to_rpy_rate;
  _a_mat(3, 9) = 1;
  _a_mat(4, 10) = 1;
  _a_mat(5, 11) = 1;
  _a_mat(11, 12) = 1;

  ab_concatenated_.block<STATE_DIM, STATE_DIM>(0, 0) = _a_mat * timestep_;

  Mat3<float> skew_mat;
  // B mat and A mat exponentials calculation
  for (int h = 0; h < planning_horizon_; h++) {
    Vec3<float> current_rpy = bodyTrajectory.block<1, 3>(h, 0).transpose();
    Mat3<float> current_robot_rot;
    rotation::rpy2rot(current_rpy, current_robot_rot);

    const Mat3<float> inv_inertia_world =
        current_robot_rot * inv_inertia_ * current_robot_rot.transpose();

    for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {
      float xPos = contactPositionsWorldFrameRotated(leg_id, h * 3);
      float yPos = contactPositionsWorldFrameRotated(leg_id, h * 3 + 1);
      float zPos = contactPositionsWorldFrameRotated(leg_id, h * 3 + 2);

      skew_mat << 0, -zPos, yPos, zPos, 0, -xPos, -yPos, xPos, 0;

      _b_mat.block<3, 3>(6, leg_id * ACTION_PER_LEG) =
          inv_inertia_world * skew_mat;

      _b_mat(9, leg_id * ACTION_PER_LEG) = 1 / _bodyMass;
      _b_mat(10, leg_id * ACTION_PER_LEG + 1) = 1 / _bodyMass;
      _b_mat(11, leg_id * ACTION_PER_LEG + 2) = 1 / _bodyMass;
    }
    // Exponentiation
    ab_concatenated_.block<STATE_DIM, ACTION_DIM>(0, STATE_DIM) =
        _b_mat * timestep_;

    FMat<float, STATE_DIM + ACTION_DIM, STATE_DIM + ACTION_DIM> ab_exp =
        ab_concatenated_.exp();

    if (h == 0) {
      _a_exp = ab_exp.block<STATE_DIM, STATE_DIM>(0, 0);
      _a_qp.block<STATE_DIM, STATE_DIM>(0, 0) = _a_exp;
    } else {
      _a_qp.block<STATE_DIM, STATE_DIM>(h * STATE_DIM, 0) =
          _a_exp * _a_qp.block<STATE_DIM, STATE_DIM>((h - 1) * STATE_DIM, 0);
    }

    _b_exps.block<STATE_DIM, ACTION_DIM>(h * STATE_DIM, 0) =
        ab_exp.block<STATE_DIM, ACTION_DIM>(0, STATE_DIM);
  }
}

void MPC::computeQpMatrices() {
  DMat<float> alphaI =
      DMat<float>::Identity(ACTION_DIM, ACTION_DIM) * MPC_ALPHA_FORCE;

  for (int i = 0; i < planning_horizon_; ++i) {
    // Diagonal block.
    _b_qp.block<STATE_DIM, ACTION_DIM>(i * STATE_DIM, i * ACTION_DIM) =
        _b_exps.block<STATE_DIM, ACTION_DIM>(i * STATE_DIM, 0);
    // Off diagonal Diagonal blocks = A^(i - j - 1) * B_exp_j.
    for (int j = 0; j < i; ++j) {
      const int power = i - j;
      if (i > 1 && j > 0) {
        _b_qp.block<STATE_DIM, ACTION_DIM>(i * STATE_DIM, j * ACTION_DIM) =
            _b_qp.block<STATE_DIM, ACTION_DIM>((i - 1) * STATE_DIM,
                                               (j - 1) * ACTION_DIM);
      } else {
        _b_qp.block<STATE_DIM, ACTION_DIM>(i * STATE_DIM, j * ACTION_DIM) =
            _a_qp.block<STATE_DIM, STATE_DIM>((power - 1) * STATE_DIM, 0) *
            _b_exps.block<STATE_DIM, ACTION_DIM>(j * STATE_DIM, 0);
      }

      b_qp_T_Q.block<ACTION_DIM, STATE_DIM>(j * ACTION_DIM, i * STATE_DIM) =
          2.0f *
          _b_qp.block<STATE_DIM, ACTION_DIM>(i * STATE_DIM, j * ACTION_DIM)
              .transpose() *
          qp_weights_block_;
    }

    b_qp_T_Q.block<ACTION_DIM, STATE_DIM>(i * ACTION_DIM, i * STATE_DIM) =
        2.0f *
        _b_qp.block<STATE_DIM, ACTION_DIM>(i * STATE_DIM, i * ACTION_DIM)
            .transpose() *
        qp_weights_block_;
  }

  _p_mat = b_qp_T_Q * _b_qp;

  for (int i = 0; i < planning_horizon_; ++i) {
    _p_mat.block<ACTION_DIM, ACTION_DIM>(i * ACTION_DIM, i * ACTION_DIM) +=
        alphaI;
  }
}

// Reset the solver so that for the next optimization run the solver is
// re-initialized.
void MPC::reset() { initial_run_ = true; }

void MPC::updateConstraints(const DMat<bool> &contactTable) {
  float fz_max =
      _bodyMass * -constants::GRAVITY_CONSTANT * CONSTRAINT_MAX_SCALE;
  float fz_min =
      _bodyMass * -constants::GRAVITY_CONSTANT * CONSTRAINT_MIN_SCALE;

  for (int i = 0; i < planning_horizon_; ++i) {
    for (int j = 0; j < MPC::NUM_LEGS; ++j) {
      const int row = (i * MPC::NUM_LEGS + j) * MPC::CONSTRAINT_DIM;

      _constraint_lb(row) = 0;
      _constraint_lb(row + 1) = 0;
      _constraint_lb(row + 2) = 0;
      _constraint_lb(row + 3) = 0;
      _constraint_lb(row + 4) = fz_min * contactTable(j, i);
      const float friction_ub = 2 * fz_max * contactTable(j, i);
      _constraint_ub(row) = friction_ub;
      _constraint_ub(row + 1) = friction_ub;
      _constraint_ub(row + 2) = friction_ub;
      _constraint_ub(row + 3) = friction_ub;
      _constraint_ub(row + 4) = fz_max * contactTable(j, i);
    }
  }

  for (int i = 0; i < planning_horizon_ * MPC::NUM_LEGS; ++i) {
    _constraint.block<MPC::CONSTRAINT_DIM, ACTION_PER_LEG>(
        i * MPC::CONSTRAINT_DIM, i * ACTION_PER_LEG)
        << -1,
        0, FRICTION_COEFF,     //
        1, 0, FRICTION_COEFF,  //
        0, -1, FRICTION_COEFF, //
        0, 1, FRICTION_COEFF,  //
        0, 0, 1;               //
  }
}

MPC::SolutionVectorType MPC::solveQP() {
  DVec<float> objective_vector = _q_vec;

  Eigen::SparseMatrix<float, Eigen::ColMajor, long long> objective_matrix =
      _p_mat.sparseView();

  Eigen::SparseMatrix<float, Eigen::ColMajor, long long> constraint_matrix =
      _constraint.sparseView();

  int num_variables = _constraint.cols();
  int num_constraints = _constraint.rows();

  ::OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = false;
  settings.scaling = 0;

  ::OSQPData data;
  data.n = num_variables;
  data.m = num_constraints;

  Eigen::SparseMatrix<float, Eigen::ColMajor, long long>
      objective_matrix_upper_triangle =
          objective_matrix.triangularView<Eigen::Upper>();

  ::csc osqp_objective_matrix = {
      objective_matrix_upper_triangle.outerIndexPtr()[num_variables],
      num_variables,
      num_variables,
      const_cast<long long *>(objective_matrix_upper_triangle.outerIndexPtr()),
      const_cast<long long *>(objective_matrix_upper_triangle.innerIndexPtr()),
      const_cast<float *>(objective_matrix_upper_triangle.valuePtr()),
      -1};

  data.P = &osqp_objective_matrix;

  ::csc osqp_constraint_matrix = {
      constraint_matrix.outerIndexPtr()[num_variables],
      num_constraints,
      num_variables,
      const_cast<long long *>(constraint_matrix.outerIndexPtr()),
      const_cast<long long *>(constraint_matrix.innerIndexPtr()),
      const_cast<float *>(constraint_matrix.valuePtr()),
      -1};

  data.A = &osqp_constraint_matrix;
  data.q = const_cast<float *>(objective_vector.data());
  data.l = _constraint_lb.data();
  data.u = _constraint_ub.data();

  const int return_code = 0;

  if (initial_run_) {
    osqp_setup(&workspace_, &data, &settings);
    initial_run_ = false;
  } else {

    c_int nnzP = objective_matrix_upper_triangle.nonZeros();

    c_int nnzA = constraint_matrix.nonZeros();

    int return_code = osqp_update_P_A(
        workspace_, objective_matrix_upper_triangle.valuePtr(), OSQP_NULL, nnzP,
        constraint_matrix.valuePtr(), OSQP_NULL, nnzA);

    return_code = osqp_update_lin_cost(workspace_, objective_vector.data());

    return_code = osqp_update_bounds(workspace_, _constraint_lb.data(),
                                     _constraint_ub.data());
  }

  if (osqp_solve(workspace_) != 0) {
    if (osqp_is_interrupted()) {
      throw std::runtime_error("MPC: solver interrupted");
    }
  }

  if (workspace_->info->status_val != OSQP_SOLVED) {
    std::cout << workspace_->info->status << std::endl;
    throw std::runtime_error("MPC: OSQP solver error");
  }

  qp_solution_ = Eigen::Map<const MPC::SolutionVectorType>(
      workspace_->solution->x, ACTION_DIM);
  /*
_u =
  DVec<float>::Map(workspace_->solution->x, ACTION_DIM * planning_horizon_);
*/
  return qp_solution_;
}

void MPC::updateObjectiveVector(const DVec<float> &currentState,
                                const DMat<float> &bodyTrajectory) {
  DMat<float> desiredStateTrajectory = bodyTrajectory.transpose();

  const DVec<float> flatDesiredStateTrajectory = Eigen::Map<const DVec<float>>(
      desiredStateTrajectory.data(), desiredStateTrajectory.size());

  _q_vec = b_qp_T_Q * (_a_qp * currentState - flatDesiredStateTrajectory);
}

MPC::SolutionVectorType
MPC::computeContactForces(const DVec<float> &currentState,
                          const DMat<bool> &contactTable,
                          const DMat<float> &contactPositionsWorldFrameRotated,
                          const DMat<float> &bodyTrajectory) {
  assert(currentState.rows() == STATE_DIM);
  assert(contactTable.rows() == NUM_LEGS);
  assert(contactTable.cols() == planning_horizon_);
  assert(contactPositionsWorldFrameRotated.rows() == NUM_LEGS);
  assert(contactPositionsWorldFrameRotated.cols() == planning_horizon_ * 3);
  assert(bodyTrajectory.rows() == planning_horizon_);
  assert(bodyTrajectory.cols() == STATE_DIM);

  computeABExponentials(currentState, contactPositionsWorldFrameRotated,
                        bodyTrajectory);
  computeQpMatrices();
  updateObjectiveVector(currentState, bodyTrajectory);
  updateConstraints(contactTable);
  solveQP();
  /*
  DVec<float> state_horizon = _a_qp * currentState + _b_qp * _u;
  */
  return qp_solution_;
}
} // namespace control
} // namespace strelka