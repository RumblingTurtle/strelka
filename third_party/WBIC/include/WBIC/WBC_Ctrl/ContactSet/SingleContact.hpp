#ifndef Cheetah_SINGLE_CONTACT
#define Cheetah_SINGLE_CONTACT

#include <WBIC/Dynamics/FloatingBaseModel.h>
#include <WBIC/Dynamics/Quadruped.h>
#include <WBIC/WBC/ContactSpec.hpp>

template <typename T> class SingleContact : public ContactSpec<T> {
public:
  SingleContact(const FloatingBaseModel<T> *robot, int contact_pt, float mu,
                float max_fz);
  virtual ~SingleContact();

  void setMaxFz(T max_fz) { _max_Fz = max_fz; }

protected:
  T _max_Fz;
  int _contact_pt;
  int _dim_U;

  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  const FloatingBaseModel<T> *robot_sys_;
};

#endif
