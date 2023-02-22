
#include <control/OldMPC.hpp>

Matrix3d ConvertRpyToRot(const Vector3d &rpy) {
  assert(rpy.size() == k3Dim);
  const AngleAxisd roll(rpy[0], Vector3d::UnitX());
  const AngleAxisd pitch(rpy[1], Vector3d::UnitY());
  const AngleAxisd yaw(rpy[2], Vector3d::UnitZ());
  Quaterniond q = yaw * pitch * roll;

  return q.matrix();
}

void ConvertToSkewSymmetric(const Vector3d &vec, Matrix3d &skew_symm) {
  skew_symm << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
}

void CalculateAMat(const Vector3d &rpy, MatrixXd *a_mat_ptr) {
  // The transformation of angular velocity to roll pitch yaw rate. Caveat:
  // rpy rate is not a proper vector and does not follow the common vector
  // transformation dicted by the rotation matrix. Here we assume the input
  // rotation is in X->Y->Z order in the extrinsic/fixed frame, or z->y'->x''
  // order in the intrinsic frame.
  const double cos_yaw = cos(rpy[2]);
  const double sin_yaw = sin(rpy[2]);
  const double cos_pitch = cos(rpy[1]);
  const double tan_pitch = tan(rpy[1]);
  Matrix3d angular_velocity_to_rpy_rate;
  angular_velocity_to_rpy_rate << cos_yaw / cos_pitch, sin_yaw / cos_pitch, 0,
      -sin_yaw, cos_yaw, 0, cos_yaw * tan_pitch, sin_yaw * tan_pitch, 1;

  MatrixXd &a_mat = *a_mat_ptr;
  a_mat.block<3, 3>(0, 6) = angular_velocity_to_rpy_rate;
  a_mat(3, 9) = 1;
  a_mat(4, 10) = 1;
  a_mat(5, 11) = 1;
  a_mat(11, 12) = 1;
}

void CalculateBMat(double inv_mass, const Matrix3d &inv_inertia,
                   const MatrixXd &foot_positions, MatrixXd *b_mat_ptr) {
  // b_mat contains non_zero elements only in row 6:12.
  const int num_legs = foot_positions.rows();
  MatrixXd &b_mat = *b_mat_ptr;
  Matrix3d skew_mat;
  for (int i = 0; i < num_legs; ++i) {
    ConvertToSkewSymmetric(foot_positions.row(i), skew_mat);
    b_mat.block<k3Dim, k3Dim>(6, i * k3Dim) = inv_inertia * skew_mat;
    b_mat(9, i * k3Dim) = inv_mass;
    b_mat(10, i * k3Dim + 1) = inv_mass;
    b_mat(11, i * k3Dim + 2) = inv_mass;
  }
}

void CalculateExponentials(const MatrixXd &a_mat, const MatrixXd &b_mat,
                           double timestep, MatrixXd *ab_mat_ptr,
                           MatrixXd *a_exp_ptr, MatrixXd *b_exp_ptr) {
  const int state_dim = 13;
  MatrixXd &ab_mat = *ab_mat_ptr;
  ab_mat.block<state_dim, state_dim>(0, 0) = a_mat * timestep;
  const int action_dim = b_mat.cols();
  ab_mat.block(0, state_dim, state_dim, action_dim) = b_mat * timestep;

  // This temporary is inevitable.
  MatrixXd ab_exp = ab_mat.exp();
  *a_exp_ptr = ab_exp.block<state_dim, state_dim>(0, 0);
  *b_exp_ptr = ab_exp.block(0, state_dim, state_dim, action_dim);
}

void CalculateQpMats(const MatrixXd &a_exp, const MatrixXd &b_exp,
                     const MatrixXd &qp_weights_single,
                     const MatrixXd &alpha_single, int horizon,
                     MatrixXd *a_qp_ptr, MatrixXd *anb_aux_ptr,
                     MatrixXd *b_qp_ptr, MatrixXd *p_mat_ptr) {
  const int state_dim = 13;
  MatrixXd &a_qp = *a_qp_ptr;
  a_qp.block(0, 0, state_dim, state_dim) = a_exp;
  for (int i = 1; i < horizon; ++i) {
    a_qp.block<state_dim, state_dim>(i * state_dim, 0) =
        a_exp * a_qp.block<state_dim, state_dim>((i - 1) * state_dim, 0);
  }

  const int action_dim = b_exp.cols();

  MatrixXd &anb_aux = *anb_aux_ptr;
  // Compute auxiliary matrix: [B_exp, A_exp * B_exp, ..., A_exp^(h-1) * B_exp]
  anb_aux.block(0, 0, state_dim, action_dim) = b_exp;
  for (int i = 1; i < horizon; ++i) {
    anb_aux.block(i * state_dim, 0, state_dim, action_dim) =
        a_exp * anb_aux.block((i - 1) * state_dim, 0, state_dim, action_dim);
  }

  MatrixXd &b_qp = *b_qp_ptr;
  for (int i = 0; i < horizon; ++i) {
    // Diagonal block.
    b_qp.block(i * state_dim, i * action_dim, state_dim, action_dim) = b_exp;
    // Off diagonal Diagonal block = A^(i - j - 1) * B_exp.
    for (int j = 0; j < i; ++j) {
      const int power = i - j;
      b_qp.block(i * state_dim, j * action_dim, state_dim, action_dim) =
          anb_aux.block(power * state_dim, 0, state_dim, action_dim);
    }
  }

  // We construct the P matrix by filling in h x h submatrices, each with size
  // action_dim x action_dim.
  // The r_th (r in [1, h]) diagonal submatrix of P is:
  // 2 * sum_{i=0:h-r}(B'A'^i L A^i B) + alpha, where h is the horizon.
  // The off-diagonal submatrix at row r and column c of P is:
  // 2 * sum_{i=0:h-c}(B'A'^{h-r-i} L A^{h-c-i} B)
  MatrixXd &p_mat = *p_mat_ptr;
  // We first compute the submatrices at column h.
  for (int i = horizon - 1; i >= 0; --i) {
    p_mat.block(i * action_dim, (horizon - 1) * action_dim, action_dim,
                action_dim) =
        anb_aux.block((horizon - i - 1) * state_dim, 0, state_dim, action_dim)
            .transpose() *
        qp_weights_single * b_exp;
    // Fill the lower-triangle part by transposing the corresponding
    // upper-triangle part.
    if (i != horizon - 1) {
      p_mat.block((horizon - 1) * action_dim, i * action_dim, action_dim,
                  action_dim) =
          p_mat
              .block(i * action_dim, (horizon - 1) * action_dim, action_dim,
                     action_dim)
              .transpose();
    }
  }

  // We then fill in the submatrices in the middle by propagating the values
  // from lower right to upper left.
  for (int i = horizon - 2; i >= 0; --i) {
    // Diagonal block.
    p_mat.block(i * action_dim, i * action_dim, action_dim, action_dim) =
        p_mat.block((i + 1) * action_dim, (i + 1) * action_dim, action_dim,
                    action_dim) +
        anb_aux.block((horizon - i - 1) * state_dim, 0, state_dim, action_dim)
                .transpose() *
            qp_weights_single *
            anb_aux.block((horizon - i - 1) * state_dim, 0, state_dim,
                          action_dim);
    // Off diagonal block
    for (int j = i + 1; j < horizon - 1; ++j) {
      p_mat.block(i * action_dim, j * action_dim, action_dim, action_dim) =
          p_mat.block((i + 1) * action_dim, (j + 1) * action_dim, action_dim,
                      action_dim) +
          anb_aux.block((horizon - i - 1) * state_dim, 0, state_dim, action_dim)
                  .transpose() *
              qp_weights_single *
              anb_aux.block((horizon - j - 1) * state_dim, 0, state_dim,
                            action_dim);
      // Fill the lower-triangle part by transposing the corresponding
      // upper-triangle part.
      p_mat.block(j * action_dim, i * action_dim, action_dim, action_dim) =
          p_mat.block(i * action_dim, j * action_dim, action_dim, action_dim)
              .transpose();
    }
  }

  // Multiply by 2 and add alpha.
  p_mat *= 2.0;
  for (int i = 0; i < horizon; ++i) {
    p_mat.block(i * action_dim, i * action_dim, action_dim, action_dim) +=
        alpha_single;
  }
}

void UpdateConstraintsMatrix(std::vector<double> &friction_coeff, int horizon,
                             int num_legs, MatrixXd *constraint_ptr) {
  const int constraint_dim = Mpc::kConstraintDim;
  MatrixXd &constraint = *constraint_ptr;
  for (int i = 0; i < horizon * num_legs; ++i) {
    constraint.block<constraint_dim, k3Dim>(i * constraint_dim, i * k3Dim)
        << -1,
        0, friction_coeff[0], 1, 0, friction_coeff[1], 0, -1, friction_coeff[2],
        0, 1, friction_coeff[3], 0, 0, 1;
  }
}

void CalculateConstraintBounds(const MatrixXd &contact_state, double fz_max,
                               double fz_min, double friction_coeff,
                               int horizon, VectorXd *constraint_lb_ptr,
                               VectorXd *constraint_ub_ptr) {
  const int constraint_dim = Mpc::kConstraintDim;

  const int num_legs = contact_state.cols();

  VectorXd &constraint_lb = *constraint_lb_ptr;
  VectorXd &constraint_ub = *constraint_ub_ptr;
  for (int i = 0; i < horizon; ++i) {
    for (int j = 0; j < num_legs; ++j) {
      const int row = (i * num_legs + j) * constraint_dim;
      constraint_lb(row) = 0;
      constraint_lb(row + 1) = 0;
      constraint_lb(row + 2) = 0;
      constraint_lb(row + 3) = 0;
      constraint_lb(row + 4) = fz_min * contact_state(i, j);

      const double friction_ub =
          (friction_coeff + 1) * fz_max * contact_state(i, j);
      constraint_ub(row) = friction_ub;
      constraint_ub(row + 1) = friction_ub;
      constraint_ub(row + 3) = friction_ub;
      constraint_ub(row + 4) = fz_max * contact_state(i, j);
    }
  }
}

double EstimateCoMHeightSimple(const MatrixXd &foot_positions_world,
                               const std::vector<int> foot_contact_states) {
  int legs_in_contact = 0;
  double com_height = 0;
  const int z_dim = 2;
  for (int i = 0; i < foot_contact_states.size(); ++i) {
    if (foot_contact_states[i]) {
      com_height += foot_positions_world(i, z_dim);
      legs_in_contact += 1;
    }
  }

  // We don't support jumping in air for now.
  DCHECK_GT(legs_in_contact, 0);
  return abs(com_height / legs_in_contact);
}

void Mpc::CalculateABExponentials() {
  const int state_dim = 13;
  // A mat calculation
  float avg_yaw = 0;
  for (int i = 0; i < planning_horizon_; i++)
    avg_yaw += desired_states_(i * 13 + 2) / planning_horizon_;
  VectorXd current_foot_state_(4);
  current_foot_state_.setZero();
  // The transformation of angular velocity to roll pitch yaw rate. Caveat:
  // rpy rate is not a proper vector and does not follow the common vector
  // transformation dicted by the rotation matrix. Here we assume the input
  // rotation is in X->Y->Z order in the extrinsic/fixed frame, or z->y'->x''
  // order in the intrinsic frame.
  const double cos_yaw = cos(avg_yaw);
  const double sin_yaw = sin(avg_yaw);
  const double cos_pitch = cos(state_[1]);
  const double tan_pitch = tan(state_[1]);
  Matrix3d angular_velocity_to_rpy_rate;
  angular_velocity_to_rpy_rate << cos_yaw / cos_pitch, sin_yaw / cos_pitch, 0,
      -sin_yaw, cos_yaw, 0, cos_yaw * tan_pitch, sin_yaw * tan_pitch, 1;

  a_mat_.block<3, 3>(0, 6) = angular_velocity_to_rpy_rate;
  a_mat_(3, 9) = 1;
  a_mat_(4, 10) = 1;
  a_mat_(5, 11) = 1;
  a_mat_(11, 12) = 1;
  ab_concatenated_.block<state_dim, state_dim>(0, 0) = a_mat_ * timestep_;

  const int action_dim = b_mat_.cols();

  Matrix3d skew_mat;
  // B mat and A mat exponentials calculation
  for (int h = 0; h < planning_horizon_; h++) {
    Vector3d current_robot_rpy = desired_states_.block(h * 13, 0, 3, 1);
    MatrixXd current_robot_rot = ConvertRpyToRot(current_robot_rpy);

    const Matrix3d inv_inertia_world =
        current_robot_rot * inv_inertia_ * current_robot_rot.transpose();
    for (int leg_id = 0; leg_id < num_legs_; leg_id++) {
      const Vector3d foot_pos = foot_table.block(h * 3, leg_id, 3, 1);
      ConvertToSkewSymmetric(foot_pos, skew_mat);
      current_foot_state_(leg_id) = contact_states_(h, leg_id);

      b_mat_.block<k3Dim, k3Dim>(6, leg_id * k3Dim) =
          inv_inertia_world * skew_mat;

      b_mat_(9, leg_id * k3Dim) = inv_mass_;
      b_mat_(10, leg_id * k3Dim + 1) = inv_mass_;
      b_mat_(11, leg_id * k3Dim + 2) = inv_mass_;
    }

    // Exponentiation
    ab_concatenated_.block(0, state_dim, state_dim, action_dim) =
        b_mat_ * timestep_;

    MatrixXd ab_exp = ab_concatenated_.exp();
    if (h == 0) {
      a_exp_ = ab_exp.block<state_dim, state_dim>(0, 0);
      b_exp_ = ab_exp.block(0, state_dim, state_dim, action_dim);
    }

    b_exps_.block(h * state_dim, 0, state_dim, action_dim) =
        ab_exp.block(0, state_dim, state_dim, action_dim);
  }
}

MatrixXd AsBlockDiagonalMat(const std::vector<double> &qp_weights,
                            int planning_horizon) {
  const Eigen::Map<const VectorXd> qp_weights_vec(qp_weights.data(),
                                                  qp_weights.size());
  // Directly return the rhs will cause a TSAN failure, probably due to the
  // asDiagonal not reall copying the memory. Creates the temporary will ensure
  // copy on return.
  const MatrixXd qp_weights_mat =
      qp_weights_vec.replicate(planning_horizon, 1).asDiagonal();
  return qp_weights_mat;
}

Mpc::Mpc(double mass, const std::vector<double> &inertia, int num_legs,
         int planning_horizon, double timestep,
         const std::vector<double> &qp_weights, double alpha)
    : mass_(mass), inv_mass_(1 / mass), inertia_(inertia.data()),
      inv_inertia_(inertia_.inverse()), num_legs_(num_legs),
      planning_horizon_(planning_horizon), timestep_(timestep),
      qp_weights_(AsBlockDiagonalMat(qp_weights, planning_horizon)),
      qp_weights_single_(AsBlockDiagonalMat(qp_weights, 1)),
      alpha_(alpha * MatrixXd::Identity(num_legs * planning_horizon * k3Dim,
                                        num_legs * planning_horizon * k3Dim)),
      alpha_single_(alpha *
                    MatrixXd::Identity(num_legs * k3Dim, num_legs * k3Dim)),
      action_dim_(num_legs * k3Dim), state_(13),
      desired_states_(13 * planning_horizon),
      contact_states_(planning_horizon, num_legs),
      foot_positions_base_(num_legs, k3Dim),
      foot_positions_world_(num_legs, k3Dim), foot_friction_coeff_(num_legs_),
      a_mat_(13, 13), b_mat_(13, action_dim_),
      ab_concatenated_(13 + action_dim_, 13 + action_dim_), a_exp_(13, 13),
      b_exp_(13, action_dim_), a_qp_(13 * planning_horizon, 13),
      b_qp_(13 * planning_horizon, action_dim_ * planning_horizon),
      p_mat_(num_legs * planning_horizon * k3Dim,
             num_legs * planning_horizon * k3Dim),
      p_mat_new_(num_legs * planning_horizon * k3Dim,
                 num_legs * planning_horizon * k3Dim),
      a_qp_new_(13 * planning_horizon, 13),
      b_qp_new_(13 * planning_horizon, action_dim_ * planning_horizon),
      q_vec_(num_legs * planning_horizon * k3Dim),
      anb_aux_(13 * planning_horizon, action_dim_),
      constraint_(kConstraintDim * num_legs * planning_horizon,
                  action_dim_ * planning_horizon),
      constraint_lb_(kConstraintDim * num_legs * planning_horizon),
      constraint_ub_(kConstraintDim * num_legs * planning_horizon),
      qp_solution_(k3Dim * num_legs * planning_horizon), workspace_(0),
      initial_run_(true), b_exps_(planning_horizon * 13, action_dim_),
      foot_table(num_legs, planning_horizon * 3) {
  assert(qp_weights.size() == 13);
  // We assume the input inertia is a 3x3 matrix.
  assert(inertia.size() == k3Dim * k3Dim);
  state_.setZero();
  desired_states_.setZero();
  contact_states_.setZero();
  foot_positions_base_.setZero();
  foot_positions_world_.setZero();
  foot_friction_coeff_.setZero();
  a_mat_.setZero();
  b_mat_.setZero();
  foot_table.setZero();
  ab_concatenated_.setZero();
  a_exp_.setZero();
  b_exp_.setZero();
  a_qp_.setZero();
  b_qp_.setZero();
  a_qp_new_.setZero();
  b_qp_new_.setZero();
  constraint_.setZero();
  constraint_lb_.setZero();
  constraint_ub_.setZero();
  b_exps_.setZero();
}

void Mpc::CalculateQpMatsProper() {
  const int state_dim = 13;
  MatrixXd &a_qp = a_qp_;
  MatrixXd &p_mat = p_mat_;
  MatrixXd &b_qp = b_qp_;

  const int action_dim = b_exp_.cols();
  a_qp.block<state_dim, state_dim>(0, 0) = a_exp_;
  for (int i = 1; i < planning_horizon_; ++i) {
    a_qp.block<state_dim, state_dim>(i * state_dim, 0) =
        a_exp_ * a_qp.block<state_dim, state_dim>((i - 1) * state_dim, 0);
  }

  for (int i = 0; i < planning_horizon_; ++i) {
    // Diagonal block.
    b_qp.block(i * state_dim, i * action_dim, state_dim, action_dim) =
        b_exps_.block(i * state_dim, 0, state_dim, action_dim);
    // Off diagonal Diagonal blocks = A^(i - j - 1) * B_exp.
    for (int j = 0; j < i; ++j) {
      const int power = i - j;
      b_qp.block(i * state_dim, j * action_dim, state_dim, action_dim) =
          a_qp.block<state_dim, state_dim>((power - 1) * state_dim, 0) *
          b_exps_.block(j * state_dim, 0, state_dim, action_dim);
    }
  }

  p_mat = b_qp.transpose() * qp_weights_ * b_qp;

  // Multiply by 2 and add alpha.
  p_mat *= 2.0;
  for (int i = 0; i < planning_horizon_; ++i) {
    p_mat.block(i * action_dim, i * action_dim, action_dim, action_dim) +=
        alpha_single_;
  }
}

void Mpc::ResetSolver() { initial_run_ = true; }

std::vector<double> &
Mpc::ComputeContactForces(MatrixXd &_mpc_table, MatrixXd &_foot_table,
                          Eigen::VectorXd &_desired_states,
                          Eigen::VectorXd &_current_state,
                          std::vector<double> &foot_friction_coeffs,
                          double kMaxScale, double kMinScale) {

  static std::vector<double> error_result;

  contact_states_ = _mpc_table.transpose();
  foot_table = _foot_table.transpose();
  desired_states_ = _desired_states;
  state_ = _current_state;

  CalculateABExponentials();

  CalculateQpMatsProper();

  const MatrixXd state_diff = a_qp_ * state_ - desired_states_;

  q_vec_ = 2 * b_qp_.transpose() * (qp_weights_ * state_diff);

  CalculateConstraintBounds(contact_states_, mass_ * kGravity * kMaxScale,
                            mass_ * kGravity * kMinScale,
                            foot_friction_coeffs[0], planning_horizon_,
                            &constraint_lb_, &constraint_ub_);

  UpdateConstraintsMatrix(foot_friction_coeffs, planning_horizon_, num_legs_,
                          &constraint_);
  foot_friction_coeff_ << foot_friction_coeffs[0], foot_friction_coeffs[1],
      foot_friction_coeffs[2], foot_friction_coeffs[3];

  Eigen::SparseMatrix<double, Eigen::ColMajor, qp_int64> objective_matrix =
      p_mat_.sparseView();
  Eigen::VectorXd objective_vector = q_vec_;
  Eigen::SparseMatrix<double, Eigen::ColMajor, qp_int64> constraint_matrix =
      constraint_.sparseView();

  int num_variables = constraint_.cols();
  int num_constraints = constraint_.rows();

  ::OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = false;
  settings.warm_start = true;
  settings.polish = true;
  settings.adaptive_rho_interval = 25;
  settings.eps_abs = 1e-3;
  settings.eps_rel = 1e-3;

  assert(p_mat_.cols() == num_variables);
  assert(p_mat_.rows() == num_variables);
  assert(q_vec_.size() == num_variables);
  assert(constraint_lb_.size() == num_constraints);
  assert(constraint_ub_.size() == num_constraints);

  VectorXd clipped_lower_bounds = constraint_lb_.cwiseMax(-OSQP_INFTY);
  VectorXd clipped_upper_bounds = constraint_ub_.cwiseMin(OSQP_INFTY);

  ::OSQPData data;
  data.n = num_variables;
  data.m = num_constraints;

  Eigen::SparseMatrix<double, Eigen::ColMajor, qp_int64>
      objective_matrix_upper_triangle =
          objective_matrix.triangularView<Eigen::Upper>();

  ::csc osqp_objective_matrix = {
      objective_matrix_upper_triangle.outerIndexPtr()[num_variables],
      num_variables,
      num_variables,
      const_cast<qp_int64 *>(objective_matrix_upper_triangle.outerIndexPtr()),
      const_cast<qp_int64 *>(objective_matrix_upper_triangle.innerIndexPtr()),
      const_cast<double *>(objective_matrix_upper_triangle.valuePtr()),
      -1};
  data.P = &osqp_objective_matrix;

  ::csc osqp_constraint_matrix = {
      constraint_matrix.outerIndexPtr()[num_variables],
      num_constraints,
      num_variables,
      const_cast<qp_int64 *>(constraint_matrix.outerIndexPtr()),
      const_cast<qp_int64 *>(constraint_matrix.innerIndexPtr()),
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

    UpdateConstraintsMatrix(foot_friction_coeffs, planning_horizon_, num_legs_,
                            &constraint_);
    foot_friction_coeff_ << foot_friction_coeffs[0], foot_friction_coeffs[1],
        foot_friction_coeffs[2], foot_friction_coeffs[3];

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

  Map<VectorXd> solution(qp_solution_.data(), qp_solution_.size());
  solution = -Map<const VectorXd>(workspace_->solution->x, workspace_->data->n);

  return qp_solution_;
}
