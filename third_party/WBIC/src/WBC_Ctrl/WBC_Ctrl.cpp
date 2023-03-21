#include <WBIC/Math/orientation_tools.h>
#include <WBIC/Utilities/pretty_print.h>
#include <WBIC/WBC_Ctrl/WBC_Ctrl.hpp>
#include <chrono>

template <typename T>
WBC_Ctrl<T>::WBC_Ctrl(FloatingBaseModel<T> model)
    : _full_config(cheetah::num_act_joint + 7), _tau_ff(cheetah::num_act_joint),
      _des_jpos(cheetah::num_act_joint), _des_jvel(cheetah::num_act_joint) {
  _iter = 0;
  _full_config.setZero();

  _model = model;
  _kin_wbc = new KinWBC<T>(cheetah::dim_config);

  _wbic = new WBIC<T>(cheetah::dim_config, &(_contact_list), &(_task_list));
  _wbic_data = new WBIC_ExtraData<T>();

  _wbic_data->_W_floating = DVec<T>::Constant(6, 0.1);
  //_wbic_data->_W_floating = DVec<T>::Constant(6, 50.);
  //_wbic_data->_W_floating[5] = 0.1;
  _wbic_data->_W_rf = DVec<T>::Constant(12, 1.);

  _Kp_joint.resize(cheetah::num_leg_joint, 5.);
  _Kd_joint.resize(cheetah::num_leg_joint, 1.5);

  //_Kp_joint_swing.resize(cheetah::num_leg_joint, 10.);
  //_Kd_joint_swing.resize(cheetah::num_leg_joint, 1.5);

  _state.q = DVec<T>::Zero(cheetah::num_act_joint);
  _state.qd = DVec<T>::Zero(cheetah::num_act_joint);
}

template <typename T> WBC_Ctrl<T>::~WBC_Ctrl() {
  delete _kin_wbc;
  delete _wbic;
  delete _wbic_data;

  _task_list.clear();
  _contact_list.clear();
}

template <typename T>
void WBC_Ctrl<T>::update(
    Quat<T> bodyOrientation, Vec3<T> bodyPosition, Vec3<T> angularVelocity,
    Vec3<T> linearVelocity,
    DVec<T> q,  // Vec12
    DVec<T> qd, // Vec12
    Vec3<T> desiredBodyRPY, Vec3<T> desiredBodyAngularVelocity,
    Vec3<T> desiredBodyPosition, Vec3<T> desiredBodyVelocity,
    Vec3<T> desiredBodyAcceleration, Vec12<T> desiredFootPosition,
    Vec12<T> desiredFootVelocity, Vec12<T> desiredFootAcceleration,
    Vec4<T> desiredContactState, Vec12<T> desiredFootForceWorld) {
  ++_iter;

  Quat<T> oriInverseQuat =
      rotationMatrixToQuaternion(quaternionToRotationMatrix(bodyOrientation));

  _UpdateModel(oriInverseQuat, bodyPosition, angularVelocity, linearVelocity, q,
               qd);
  // auto t_end = std::chrono::high_resolution_clock::now();
  // double elapsed_time_ms = std::chrono::duration<double,
  // std::milli>(t_end-t_start).count(); std::cout << "Update model, ms: "<<
  // elapsed_time_ms << endl;

  // Task & Contact Update
  _ContactTaskUpdate(desiredBodyRPY, desiredBodyAngularVelocity,
                     desiredBodyPosition, desiredBodyVelocity,
                     desiredBodyAcceleration, desiredFootPosition,
                     desiredFootVelocity, desiredFootAcceleration,
                     desiredContactState, desiredFootForceWorld);

  // Kin WBC Computation
  // auto t_start2 = std::chrono::high_resolution_clock::now();
  _kin_wbc->FindConfiguration(_full_config, _task_list, _contact_list,
                              _des_jpos, _des_jvel);
  // auto t_end2 = std::chrono::high_resolution_clock::now();
  // double elapsed_time_ms2 = std::chrono::duration<double,
  // std::milli>(t_end2-t_start2).count(); std::cout << "Find q, dq, ms: "<<
  // elapsed_time_ms2 << endl;
}

template <typename T> void WBC_Ctrl<T>::compute_tau() {
  // WBIC
  // auto t_start3 = std::chrono::high_resolution_clock::now();
  _wbic->UpdateSetting(_A, _Ainv, _coriolis, _grav);
  _wbic->MakeTorque(_tau_ff, _wbic_data);
  // auto t_end3 = std::chrono::high_resolution_clock::now();
  // double elapsed_time_ms3 = std::chrono::duration<double,
  // std::milli>(t_end3-t_start3).count(); std::cout << "Compute torque, ms: "<<
  // elapsed_time_ms3 << endl;
}

template <typename T>
void WBC_Ctrl<T>::_UpdateModel(Quat<T> bodyOrientation, Vec3<T> bodyPosition,
                               Vec3<T> angularVelocity,
                               Vec3<T> linearVelocity, // Vec6
                               DVec<T> q,              // Vec12
                               DVec<T> qd              // Vec12
) {
  _state.bodyOrientation = bodyOrientation;
  _state.bodyPosition = bodyPosition;
  for (size_t i(0); i < 3; ++i) {
    _state.bodyVelocity[i] = angularVelocity[i];
    _state.bodyVelocity[i + 3] = linearVelocity[i];

    for (size_t leg(0); leg < 4; ++leg) {
      _state.q[3 * leg + i] = q[3 * leg + i];
      _state.qd[3 * leg + i] = qd[3 * leg + i];

      _full_config[3 * leg + i + 6] = _state.q[3 * leg + i];
    }
  }
  _model.setState(_state);

  _model.contactJacobians();
  _model.massMatrix();
  _model.generalizedGravityForce();
  _model.generalizedCoriolisForce();

  _A = _model.getMassMatrix();
  _grav = _model.getGravityForce();
  _coriolis = _model.getCoriolisForce();
  _Ainv = _A.inverse();

  // pretty_print(bodyOrientation, std::cout, "Body Orientation");
  // pretty_print(bodyPosition, std::cout, "Body Position");
  // pretty_print(q, std::cout, "q vector");
  // pretty_print(_A, std::cout, "A matrix");
  // pretty_print(_grav, std::cout, "g vector");
}

template <typename T>
void WBC_Ctrl<T>::getFootPosWorldFrame(Eigen::Matrix<T, 4, 3> footPositions) {
  for (int legId = 0; legId < 4; legId++) {
    footPositions(legId, 0) = _model._pGC[9 + 2 * legId][0];
    footPositions(legId, 1) = _model._pGC[9 + 2 * legId][1];
    footPositions(legId, 2) = _model._pGC[9 + 2 * legId][2];
  }
}
template class WBC_Ctrl<float>;