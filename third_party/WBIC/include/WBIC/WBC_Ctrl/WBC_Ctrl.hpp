#ifndef WBC_CONTROLLER_H
#define WBC_CONTROLLER_H

#include <WBIC/Dynamics/FloatingBaseModel.h>
#include <WBIC/Dynamics/Quadruped.h>
#include <WBIC/WBC/WBIC/KinWBC.hpp>
#include <WBIC/WBC/WBIC/WBIC.hpp>
#include <WBIC/cppTypes.h>
#include <vector>

#define WBCtrl WBC_Ctrl<T>

class MIT_UserParameters;

template <typename T> class WBC_Ctrl {
public:
  WBC_Ctrl(FloatingBaseModel<T> model);
  virtual ~WBC_Ctrl();

  void update(Quat<T> bodyOrientation, Vec3<T> bodyPosition,
              Vec3<T> angularVelocity,
              Vec3<T> linearVelocity, // Vec6
              DVec<T> q,              // Vec12
              DVec<T> qd,             // Vec12
              Vec3<T> desiredBodyRPY, Vec3<T> desiredBodyAngularVelocity,
              Vec3<T> desiredBodyPosition, Vec3<T> desiredBodyVelocity,
              Vec3<T> desiredBodyAcceleration, Vec12<T> desiredFootPosition,
              Vec12<T> desiredFootVelocity, Vec12<T> desiredFootAcceleration,
              Vec4<T> desiredContactState, Vec12<T> desiredFootForceWorld);

  void compute_tau();

  void setFloatingBaseWeight(const T &weight) {
    _wbic_data->_W_floating = DVec<T>::Constant(6, weight);
  }

  void setWeights(std::vector<T> float_w, std::vector<T> rf_w) {
    _wbic_data->_W_floating =
        Eigen::Map<DVec<T>>(float_w.data(), float_w.size());
    _wbic_data->_W_rf = Eigen::Map<DVec<T>>(rf_w.data(), rf_w.size());
  }

  DVec<T> getTau() {
    DVec<T> tau{_tau_ff};
    return tau;
  };

  DVec<T> get_qDes() {
    DVec<T> q{_des_jpos};
    return q;
  };

  DVec<T> get_dqDes() { return _des_jvel; };

  FloatingBaseModel<T> _model;

protected:
  virtual void
  _ContactTaskUpdate(Vec3<T> desiredBodyRPY, Vec3<T> desiredBodyAngularVelocity,
                     Vec3<T> desiredBodyPosition, Vec3<T> desiredBodyVelocity,
                     Vec3<T> desiredBodyAcceleration,
                     Vec12<T> desiredFootPosition, Vec12<T> desiredFootVelocity,
                     Vec12<T> desiredFootAcceleration,
                     Vec4<T> desiredContactState,
                     Vec12<T> desiredFootForceWorld // MPC res
                     ) = 0;

  void _UpdateModel(Quat<T> bodyOrientation, Vec3<T> bodyPosition,
                    Vec3<T> angularVelocity,
                    Vec3<T> linearVelocity, // Vec6
                    DVec<T> q,              // Vec12
                    DVec<T> qd              // Vec12
  );

  void getFootPosWorldFrame(Eigen::Matrix<T, 4, 3> footPositions);
  KinWBC<T> *_kin_wbc;
  WBIC<T> *_wbic;
  WBIC_ExtraData<T> *_wbic_data;

  std::vector<ContactSpec<T> *> _contact_list;
  std::vector<Task<T> *> _task_list;

  DMat<T> _A;
  DMat<T> _Ainv;
  DVec<T> _grav;
  DVec<T> _coriolis;

  FBModelState<T> _state;

  DVec<T> _full_config;
  DVec<T> _tau_ff;
  DVec<T> _des_jpos;
  DVec<T> _des_jvel;

  std::vector<T> _Kp_joint, _Kd_joint;
  // std::vector<T> _Kp_joint_swing, _Kd_joint_swing;

  unsigned long long _iter;
};
#endif
