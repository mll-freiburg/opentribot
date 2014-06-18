#ifndef _OMNIROBOTKIN_H_
#define _OMNIROBOTKIN_H_

/* Klasse zur Berechnung der Kinematik eines OmniDirektionalen Roboters.
 * Parameter L1, L2, delta: Anornung der Räder am Roboter (siehe Diplomarbeit Hafner 02)
 * Parameter R: Radius der Räder
 */

#include <iostream>

class OmniRobotKin
{
 protected:
  std::ostream *errout, *infoout;

  //parameter
  float L1_m;       static const float L1_m_def ;
  float L2_m;       static const float L2_m_def ;
  float R_m;        static const float R_m_def  ;
  float delta_deg;  static const float delta_deg_def ;

  float delta_rad;
  float sindelta, cosdelta;
  void init_params();
  bool read_params(const char *, const char *);

 public:
  OmniRobotKin(const char* _conf_fname, const char * _conf_chapter,
	       std::ostream *_errout = &std::cerr, 
	       std::ostream *_infoout = &std::cout);

  OmniRobotKin(float _L1_m, float _L2_m, float _delta_deg, float _R_m,
	       std::ostream *_errout = &std::cerr, 
	       std::ostream *_infoout = &std::cout);
  
  ~OmniRobotKin();
  
  void cmptInvKin_RobotFrame(float _xm, float _ym, float _phim, float & _v1,float & _v2,float & _v3);
  
  void cmptDirectKin_RobotFrame(float _v1,float _v2,float _v3, float & _xm, float & _ym, float & _phim);

  
};

#endif
