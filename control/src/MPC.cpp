
#include <common/utilities.hpp>
#include <control/MPC.hpp>
#include <iostream>
namespace strelka {

constexpr int MPC::STATE_DIM;
constexpr int MPC::CONSTRAINT_DIM;
constexpr int MPC::NUM_LEGS;
constexpr int MPC::ACTION_DIM;

MPC::MPC(double mass, const Vec3<double> &inertia, int planning_horizon,
         double timestep, const DVec<double> &qp_weights, double alpha,
         const Vec4<double> &_footFrictionCoefficients, double kMaxScale,
         double kMinScale)
    : _bodyMass(mass), inertia_(inertia.asDiagonal()),
      inv_inertia_(inertia_.inverse()), planning_horizon_(planning_horizon),
      timestep_(timestep),
      qp_weights_(qp_weights.replicate(planning_horizon, 1).asDiagonal()),
      alpha_(alpha), _currentState(STATE_DIM),
      _desiredStateTrajectory(STATE_DIM * planning_horizon),
      _contactStates(NUM_LEGS, planning_horizon), _a_mat(STATE_DIM, STATE_DIM),
      _b_mat(STATE_DIM, ACTION_DIM),
      ab_concatenated_(STATE_DIM + ACTION_DIM, STATE_DIM + ACTION_DIM),
      _a_exp(STATE_DIM, STATE_DIM), _b_exp(STATE_DIM, ACTION_DIM),
      _a_qp(STATE_DIM * planning_horizon, STATE_DIM),
      _b_qp(STATE_DIM * planning_horizon, ACTION_DIM * planning_horizon),
      _p_mat(NUM_LEGS * planning_horizon * 3, NUM_LEGS * planning_horizon * 3),
      _q_vec(NUM_LEGS * planning_horizon * 3),
      _constraint(CONSTRAINT_DIM * NUM_LEGS * planning_horizon,
                  ACTION_DIM * planning_horizon),
      _constraint_lb(CONSTRAINT_DIM * NUM_LEGS * planning_horizon),
      _constraint_ub(CONSTRAINT_DIM * NUM_LEGS * planning_horizon),
      qp_solution_(3 * NUM_LEGS * planning_horizon), workspace_(0),
      initial_run_(true), _b_exps(planning_horizon * STATE_DIM, ACTION_DIM),
      _contactPositionsBodyFrame(NUM_LEGS, planning_horizon * 3),
      kMaxScale(kMaxScale), kMinScale(kMinScale) {

  assert(qp_weights.size() == STATE_DIM);
  this->_footFrictionCoefficients = _footFrictionCoefficients;
  _currentState.setZero();
  _desiredStateTrajectory.setZero();
  _contactStates.setZero();
  _a_mat.setZero();
  _b_mat.setZero();
  _contactPositionsBodyFrame.setZero();
  ab_concatenated_.setZero();
  _a_exp.setZero();
  _b_exp.setZero();
  _a_qp.setZero();
  _b_qp.setZero();
  _constraint.setZero();
  _constraint_lb.setZero();
  _constraint_ub.setZero();
  _b_exps.setZero();
  _currentState(12) = (double)constants::GRAVITY_CONSTANT;
}

MPC::~MPC() { osqp_cleanup(workspace_); }

void MPC::computeABExponentials() {
  // A mat calculation
  double avg_yaw = 0;
  for (int i = 0; i < planning_horizon_; i++) {
    avg_yaw += _desiredStateTrajectory(i * STATE_DIM + 2) / planning_horizon_;
  }
  // The transformation of angular velocity to roll pitch yaw rate. Caveat:
  // rpy rate is not a proper vector and does not follow the common vector
  // transformation dicted by the rotation matrix. Here we assume the input
  // rotation is in X->Y->Z order in the extrinsic/fixed frame, or z->y'->x''
  // order in the intrinsic frame.
  const double cos_yaw = cos(avg_yaw);
  const double sin_yaw = sin(avg_yaw);
  const double cos_pitch = cos(_currentState[1]);
  const double tan_pitch = tan(_currentState[1]);
  Mat3<double> angular_velocity_to_rpy_rate;
  angular_velocity_to_rpy_rate << cos_yaw / cos_pitch, sin_yaw / cos_pitch, 0,
      -sin_yaw, cos_yaw, 0, cos_yaw * tan_pitch, sin_yaw * tan_pitch, 1;

  _a_mat.block<3, 3>(0, 6) = angular_velocity_to_rpy_rate;
  _a_mat(3, 9) = 1;
  _a_mat(4, 10) = 1;
  _a_mat(5, 11) = 1;
  _a_mat(11, 12) = 1;
  ab_concatenated_.block<STATE_DIM, STATE_DIM>(0, 0) = _a_mat * timestep_;

  const int ACTION_DIM = _b_mat.cols();

  Mat3<double> skew_mat;
  // B mat and A mat exponentials calculation
  for (int h = 0; h < planning_horizon_; h++) {
    Vec3<double> current_robot_rpy =
        _desiredStateTrajectory.block(h * STATE_DIM, 0, 3, 1);
    Mat3<double> current_robot_rot;
    rotation::rpy2rot(current_robot_rpy, current_robot_rot);

    const Mat3<double> inv_inertia_world =
        current_robot_rot * inv_inertia_ * current_robot_rot.transpose();
    for (int leg_id = 0; leg_id < NUM_LEGS; leg_id++) {
      Vec3<double> footPos =
          _contactPositionsBodyFrame.block(leg_id, h * 3, 1, 3).transpose();
      skew_mat << 0, -footPos(2), footPos(1), footPos(2), 0, -footPos(0),
          -footPos(1), footPos(0), 0;

      _b_mat.block<3, 3>(6, leg_id * 3) = inv_inertia_world * skew_mat;

      _b_mat(9, leg_id * 3) = 1 / _bodyMass;
      _b_mat(10, leg_id * 3 + 1) = 1 / _bodyMass;
      _b_mat(11, leg_id * 3 + 2) = 1 / _bodyMass;
    }

    // Exponentiation
    ab_concatenated_.block(0, STATE_DIM, STATE_DIM, ACTION_DIM) =
        _b_mat * timestep_;

    DMat<double> ab_exp = ab_concatenated_.exp();
    if (h == 0) {
      _a_exp = ab_exp.block<STATE_DIM, STATE_DIM>(0, 0);
      _b_exp = ab_exp.block(0, STATE_DIM, STATE_DIM, ACTION_DIM);
    }

    _b_exps.block(h * STATE_DIM, 0, STATE_DIM, ACTION_DIM) =
        ab_exp.block(0, STATE_DIM, STATE_DIM, ACTION_DIM);
  }
}

void MPC::computeQpMatrices() {
  _a_qp.block<STATE_DIM, STATE_DIM>(0, 0) = _a_exp;
  for (int i = 1; i < planning_horizon_; ++i) {
    _a_qp.block<STATE_DIM, STATE_DIM>(i * STATE_DIM, 0) =
        _a_exp * _a_qp.block<STATE_DIM, STATE_DIM>((i - 1) * STATE_DIM, 0);
  }

  for (int i = 0; i < planning_horizon_; ++i) {
    // Diagonal block.
    _b_qp.block(i * STATE_DIM, i * ACTION_DIM, STATE_DIM, ACTION_DIM) =
        _b_exps.block(i * STATE_DIM, 0, STATE_DIM, ACTION_DIM);
    // Off diagonal Diagonal blocks = A^(i - j - 1) * B_exp_j.
    for (int j = 0; j < i; ++j) {
      const int power = i - j;
      _b_qp.block(i * STATE_DIM, j * ACTION_DIM, STATE_DIM, ACTION_DIM) =
          _a_qp.block<STATE_DIM, STATE_DIM>((power - 1) * STATE_DIM, 0) *
          _b_exps.block(j * STATE_DIM, 0, STATE_DIM, ACTION_DIM);
    }
  }

  _p_mat = _b_qp.transpose() * qp_weights_ * _b_qp * 2;

  for (int i = 0; i < planning_horizon_ * ACTION_DIM; ++i) {
    _p_mat(i, i) += alpha_;
  }
}

// Reset the solver so that for the next optimization run the solver is
// re-initialized.
void MPC::reset() { initial_run_ = true; }

void MPC::updateConstraintsMatrix() {
  const int constraint_dim = MPC::CONSTRAINT_DIM;
  for (int i = 0; i < planning_horizon_ * NUM_LEGS; ++i) {
    _constraint.block<constraint_dim, 3>(i * constraint_dim, i * 3) << -1, 0,
        _footFrictionCoefficients[0], 1, 0, _footFrictionCoefficients[1], 0, -1,
        _footFrictionCoefficients[2], 0, 1, _footFrictionCoefficients[3], 0, 0,
        1;
  }
}

void MPC::computeConstraintBounds() {
  const int constraint_dim = MPC::CONSTRAINT_DIM;

  float fz_max = _bodyMass * -constants::GRAVITY_CONSTANT * kMaxScale;
  float fz_min = _bodyMass * -constants::GRAVITY_CONSTANT * kMinScale;
  float friction_coeff = _footFrictionCoefficients[0];

  for (int i = 0; i < planning_horizon_; ++i) {
    for (int j = 0; j < NUM_LEGS; ++j) {
      const int row = (i * NUM_LEGS + j) * constraint_dim;
      _constraint_lb(row) = 0;
      _constraint_lb(row + 1) = 0;
      _constraint_lb(row + 2) = 0;
      _constraint_lb(row + 3) = 0;
      _constraint_lb(row + 4) = fz_min * _contactStates(j, i);

      const double friction_ub =
          (friction_coeff + 1) * fz_max * _contactStates(j, i);
      _constraint_ub(row) = friction_ub;
      _constraint_ub(row + 1) = friction_ub;
      _constraint_ub(row + 3) = friction_ub;
      _constraint_ub(row + 4) = fz_max * _contactStates(j, i);
    }
  }
}

DVec<double> &MPC::solveQP() {
  static DVec<double> error_result;

  DVec<double> objective_vector = _q_vec;

  Eigen::SparseMatrix<double, Eigen::ColMajor, long long> objective_matrix =
      _p_mat.sparseView();

  Eigen::SparseMatrix<double, Eigen::ColMajor, long long> constraint_matrix =
      _constraint.sparseView();

  int num_variables = _constraint.cols();
  int num_constraints = _constraint.rows();

  ::OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = false;
  settings.warm_start = true;
  settings.polish = true;
  settings.adaptive_rho_interval = 25;
  settings.eps_abs = 1e-3;
  settings.eps_rel = 1e-3;

  DVec<double> clipped_lower_bounds = _constraint_lb.cwiseMax(-OSQP_INFTY);
  DVec<double> clipped_upper_bounds = _constraint_ub.cwiseMin(OSQP_INFTY);

  ::OSQPData data;
  data.n = num_variables;
  data.m = num_constraints;

  Eigen::SparseMatrix<double, Eigen::ColMajor, long long>
      objective_matrix_upper_triangle =
          objective_matrix.triangularView<Eigen::Upper>();

  ::csc osqp_objective_matrix = {
      objective_matrix_upper_triangle.outerIndexPtr()[num_variables],
      num_variables,
      num_variables,
      const_cast<long long *>(objective_matrix_upper_triangle.outerIndexPtr()),
      const_cast<long long *>(objective_matrix_upper_triangle.innerIndexPtr()),
      const_cast<double *>(objective_matrix_upper_triangle.valuePtr()),
      -1};

  data.P = &osqp_objective_matrix;

  ::csc osqp_constraint_matrix = {
      constraint_matrix.outerIndexPtr()[num_variables],
      num_constraints,
      num_variables,
      const_cast<long long *>(constraint_matrix.outerIndexPtr()),
      const_cast<long long *>(constraint_matrix.innerIndexPtr()),
      const_cast<double *>(constraint_matrix.valuePtr()),
      -1};

  data.A = &osqp_constraint_matrix;
  data.q = const_cast<double *>(objective_vector.data());
  data.l = clipped_lower_bounds.data();
  data.u = clipped_upper_bounds.data();

  const int return_code = 0;

  if (workspace_ == 0) {
    osqp_setup(&workspace_, &data, &settings);
    initial_run_ = false;
  } else {

    updateConstraintsMatrix();

    c_int nnzP = objective_matrix_upper_triangle.nonZeros();

    c_int nnzA = constraint_matrix.nonZeros();

    int return_code = osqp_update_P_A(
        workspace_, objective_matrix_upper_triangle.valuePtr(), OSQP_NULL, nnzP,
        constraint_matrix.valuePtr(), OSQP_NULL, nnzA);

    return_code = osqp_update_lin_cost(workspace_, objective_vector.data());

    return_code = osqp_update_bounds(workspace_, clipped_lower_bounds.data(),
                                     clipped_upper_bounds.data());
  }

  if (osqp_solve(workspace_) != 0) {
    if (osqp_is_interrupted()) {
      return error_result;
    }
  }

  if (workspace_->info->status_val != OSQP_SOLVED) {
    return error_result;
  }

  DVec<double> solution(3 * NUM_LEGS * planning_horizon_);
  solution = -Eigen::Map<const DVec<double>>(workspace_->solution->x,
                                             workspace_->data->n);

  return qp_solution_;
}

void MPC::updateObjectiveVector() {
  const DMat<double> state_diff =
      _a_qp * _currentState - _desiredStateTrajectory;

  _q_vec = 2 * _b_qp.transpose() * (qp_weights_ * state_diff);
}

DVec<double> &MPC::computeContactForces(robots::Robot &robot,
                                        DMat<bool> &contactTable,
                                        DMat<float> &contactPositionsBodyFrame,
                                        DMat<float> &bodyTrajectory) {

  // DVec<double> &currentState;
  _currentState.block(0, 0, 3, 1) = robot.currentRPY().cast<double>();
  _currentState.block(3, 0, 3, 1) = robot.positionWorldFrame().cast<double>();
  _currentState.block(6, 0, 3, 1) =
      robot.rotateBodyToWorldFrame(robot.gyroscopeBodyFrame()).cast<double>();
  _currentState.block(9, 0, 3, 1) =
      robot.rotateBodyToWorldFrame(robot.linearVelocityBodyFrame())
          .cast<double>();

  _contactStates = contactTable;
  _contactPositionsBodyFrame = contactPositionsBodyFrame.cast<double>();
  _desiredStateTrajectory =
      Eigen::Map<DVec<float>>(bodyTrajectory.data(), bodyTrajectory.size())
          .cast<double>();

  computeABExponentials();

  computeQpMatrices();

  computeConstraintBounds();

  updateConstraintsMatrix();

  return solveQP();
}
} // namespace strelka