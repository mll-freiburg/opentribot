#ifndef _SINGLEDRIVE_H_
#define _SINGLEDRIVE_H_

#include <iostream>

class SingleDrive
{

 protected:
  std::ostream *errout, *infoout;
  
  static const int num_of_enc_imp_default;
  static const int max_motor_vel_default ;
  static const int delta_dist_default    ;
  static const int C_nom_default         ;
  static const int C_max_default         ;
  static const int kp_default            ;
  static const int ki_default            ;
  static const int kd_default            ;
  static const float gearparam_default   ;

  int num_of_enc_imp;    // number of Impulses on encoder disc
  int max_motor_vel;     // maximal velocity of motor in turns/min
  int delta_dist;        // a distance for 1 impulse
  
  int C_nom;             // nominal current in mA
  int C_max;             // maximal current in mA

  int kp;                // params for the pid velocity controller
  int ki;
  int kd;

  float gearparam;       // gear factor

 public:
  SingleDrive(const char* _conf_fname, const char* _conf_chapter, 
	      std::ostream *_errout = &std::cerr, std::ostream *_infoout = &std::cout);
  ~SingleDrive();

  void get_pid_params(int &_kp, int &_ki, int &_kd){_kp=kp; _ki=ki;_kd=kd;};
  void set_pid_params(int _kp, int _ki, int _kd){kp=_kp; ki=_ki;kd=_kd;};

  void get_current_params(int &_c_nom, int &_c_max){_c_nom=C_nom; _c_max=C_max;};
  void set_current_params(int _c_nom, int _c_max){C_nom=_c_nom; C_max=_c_max;};

  void get_motor_params(int &_num_imp, int &_max_vel, int &_delta_dist){ 
    _num_imp=num_of_enc_imp; _max_vel=max_motor_vel; _delta_dist=delta_dist;};
  void set_motor_params(int _num_imp, int _max_vel, int _delta_dist){ 
    num_of_enc_imp = _num_imp; 
    max_motor_vel  = _max_vel; 
    delta_dist     = _delta_dist;};

  void get_gear_param(float &_gp){_gp=gearparam;};
  void set_gear_param(float _gp){gearparam=_gp;};

  void print(std::ostream &_out);

};



#endif
