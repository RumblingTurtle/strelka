#include <WBIC/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>

template <typename T>
LocomotionCtrl<T>::LocomotionCtrl(FloatingBaseModel<T> model, T *Kp, T *Kd,
                                  T *Kp_kin, T *floating_W, T *rf_W, T _mu,
                                  T _max_fz)
    : WBC_Ctrl<T>(model) {
  for (int i = 0; i < 3; i++) {
    Kp_body[i] = Kp[i];
    Kd_body[i] = Kd[i];

    Kp_ori[i] = Kp[i + 3];
    Kd_ori[i] = Kd[i + 3];

    Kp_foot[i] = Kp[i + 6];
    Kd_foot[i] = Kd[i + 6];

    Kp_kin_pos[i] = Kp_kin[i];
    Kp_kin_ori[i] = Kp_kin[i + 3];
    Kp_kin_link[i] = Kp_kin[i + 6];
  }

  this->setWeights(std::vector<T>(floating_W, floating_W + 6),
                   std::vector<T>(rf_W, rf_W + 12));

  _body_pos_task = new BodyPosTask<T>(&(WBCtrl::_model));
  _body_ori_task = new BodyOriTask<T>(&(WBCtrl::_model));

  _foot_contact[0] =
      new SingleContact<T>(&(WBCtrl::_model), linkID::FR, _mu, _max_fz);
  _foot_contact[1] =
      new SingleContact<T>(&(WBCtrl::_model), linkID::FL, _mu, _max_fz);
  _foot_contact[2] =
      new SingleContact<T>(&(WBCtrl::_model), linkID::HR, _mu, _max_fz);
  _foot_contact[3] =
      new SingleContact<T>(&(WBCtrl::_model), linkID::HL, _mu, _max_fz);

  _foot_task[0] = new LinkPosTask<T>(&(WBCtrl::_model), linkID::FR);
  _foot_task[1] = new LinkPosTask<T>(&(WBCtrl::_model), linkID::FL);
  _foot_task[2] = new LinkPosTask<T>(&(WBCtrl::_model), linkID::HR);
  _foot_task[3] = new LinkPosTask<T>(&(WBCtrl::_model), linkID::HL);
}

template <typename T> LocomotionCtrl<T>::~LocomotionCtrl() {
  delete _body_pos_task;
  delete _body_ori_task;

  for (size_t i(0); i < 4; ++i) {
    delete _foot_contact[i];
    delete _foot_task[i];
  }
}

template <typename T>
void LocomotionCtrl<T>::_ContactTaskUpdate(
    Vec3<T> desiredBodyRPY, Vec3<T> desiredBodyAngularVelocity,
    Vec3<T> desiredBodyPosition, Vec3<T> desiredBodyVelocity,
    Vec3<T> desiredBodyAcceleration, Vec12<T> desiredFootPosition,
    Vec12<T> desiredFootVelocity, Vec12<T> desiredFootAcceleration,
    Vec4<T> desiredContactState, Vec12<T> desiredFootForceWorld) {

  _ParameterSetup();

  // Wash out the previous setup
  _CleanUp();

  _quat_des = ori::rpyToQuat(desiredBodyRPY);

  Vec3<T> zero_vec3;
  zero_vec3.setZero();
  _body_ori_task->UpdateTask(&_quat_des, desiredBodyAngularVelocity, zero_vec3);
  _body_pos_task->UpdateTask(&(desiredBodyPosition), desiredBodyVelocity,
                             desiredBodyAcceleration);

  WBCtrl::_task_list.push_back(_body_ori_task);
  WBCtrl::_task_list.push_back(_body_pos_task);

  for (size_t leg(0); leg < 4; ++leg) {
    if (desiredContactState[leg] > 0.) { // Contact
      _foot_contact[leg]->setRFDesired(
          desiredFootForceWorld.block(leg * 3, 0, 3, 1));
      _foot_contact[leg]->UpdateContactSpec();
      WBCtrl::_contact_list.push_back(_foot_contact[leg]);

    } else { // No Contact (swing)
      Vec3<T> pf = desiredFootPosition.block(leg * 3, 0, 3, 1);
      _foot_task[leg]->UpdateTask(
          &(pf), desiredFootVelocity.block(leg * 3, 0, 3, 1),
          desiredFootAcceleration.block(leg * 3, 0, 3, 1));
      // zero_vec3);
      WBCtrl::_task_list.push_back(_foot_task[leg]);
    }
  }
}

template <typename T> void LocomotionCtrl<T>::_ParameterSetup() {

  for (size_t i(0); i < 3; ++i) {
    ((BodyPosTask<T> *)_body_pos_task)->_Kp[i] = Kp_body[i];
    ((BodyPosTask<T> *)_body_pos_task)->_Kd[i] = Kd_body[i];
    ((BodyPosTask<T> *)_body_pos_task)->_Kp_kin[i] = Kp_kin_pos[i];

    ((BodyOriTask<T> *)_body_ori_task)->_Kp[i] = Kp_ori[i];
    ((BodyOriTask<T> *)_body_ori_task)->_Kd[i] = Kd_ori[i];
    ((BodyOriTask<T> *)_body_ori_task)->_Kp_kin[i] = Kp_kin_ori[i];

    for (size_t j(0); j < 4; ++j) {
      ((LinkPosTask<T> *)_foot_task[j])->_Kp[i] = Kp_foot[i];
      ((LinkPosTask<T> *)_foot_task[j])->_Kd[i] = Kd_foot[i];
      ((LinkPosTask<T> *)_foot_task[j])->_Kp_kin[i] = Kp_kin_link[i];
    }
  }
}

template <typename T> void LocomotionCtrl<T>::_CleanUp() {
  WBCtrl::_contact_list.clear();
  WBCtrl::_task_list.clear();
}

template class LocomotionCtrl<float>;
template class LocomotionCtrl<double>;
