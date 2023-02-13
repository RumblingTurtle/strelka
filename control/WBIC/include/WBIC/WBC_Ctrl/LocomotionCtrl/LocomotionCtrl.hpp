#ifndef LOCOMOTION_CONTROLLER
#define LOCOMOTION_CONTROLLER

#include <WBC_Ctrl/ContactSet/SingleContact.hpp>
#include <WBC_Ctrl/TaskSet/BodyOriTask.hpp>
#include <WBC_Ctrl/TaskSet/BodyPosTask.hpp>
#include <WBC_Ctrl/TaskSet/LinkPosTask.hpp>
#include <WBC_Ctrl/WBC_Ctrl.hpp>
#include <cppTypes.h>
#include <vector>

template <typename T> class LocomotionCtrlData {
public:
  Vec3<T> pBody_des;
  Vec3<T> vBody_des;
  Vec3<T> aBody_des;

  Vec3<T> pBody_RPY_des;
  Vec3<T> vBody_Ori_des;

  Vec3<T> pFoot_des[4];
  Vec3<T> vFoot_des[4];
  Vec3<T> aFoot_des[4];
  Vec3<T> Fr_des[4];

  Vec4<T> contact_state;
};

template <typename T> class LocomotionCtrl : public WBC_Ctrl<T> {

public:
  LocomotionCtrl(FloatingBaseModel<T> model, T *Kp, T *Kd, T *Kp_kin,
                 T *floating_W, T *rf_W, T _mu = 0.4, T _max_fz = 1500);

  virtual ~LocomotionCtrl();

  Vec3<T> Kp_body{1, 1, 1};
  Vec3<T> Kd_body{1, 1, 1};

  Vec3<T> Kp_ori{1, 1, 1};
  Vec3<T> Kd_ori{1, 1, 1};

  Vec3<T> Kp_foot{1, 1, 1};
  Vec3<T> Kd_foot{1, 1, 1};

  Vec3<T> Kp_joint{1, 1, 1};
  Vec3<T> Kd_joint{1, 1, 1};

  Vec3<T> Kp_kin_pos{1, 1, 1};
  Vec3<T> Kp_kin_ori{1, 1, 1};
  Vec3<T> Kp_kin_link{1, 1, 1};

  float mu;
  float max_fz;

protected:
  virtual void _ContactTaskUpdate(Vec3<T> pBody_RPY_des, Vec3<T> vBody_Ori_des,
                                  Vec3<T> pBody_des, Vec3<T> vBody_des,
                                  Vec3<T> aBody_des, Vec12<T> pFoot_des,
                                  Vec12<T> vFoot_des, Vec12<T> aFoot_des,
                                  Vec4<T> contact_state, Vec12<T> Fr_des_MPC);

  void _ParameterSetup();
  void _CleanUp();

  LocomotionCtrlData<T> *_input_data;

  Task<T> *_body_pos_task;
  Task<T> *_body_ori_task;

  Task<T> *_foot_task[4];
  ContactSpec<T> *_foot_contact[4];

  Vec3<T> pre_foot_vel[4];

  Vec3<T> _Fr_result[4];
  Quat<T> _quat_des;
};

#endif
