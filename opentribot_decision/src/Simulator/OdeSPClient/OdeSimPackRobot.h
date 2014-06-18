#ifndef _ODE_SIM_PACK_ROBOT_H_
#define _ODE_SIM_PACK_ROBOT_H_

#include "../../Robot/RobotType.h"
#include "../../WorldModel/WorldModel.h"
#include "../../Fundamental/ConfigReader.h"
#include "OdeSimPackClient.h"

namespace Tribots {
  class OdeSimPackRobot:public RobotType {
   
  public:
    OdeSimPackRobot(const ConfigReader & vr) throw(TribotsException, std::bad_alloc);
    ~OdeSimPackRobot() throw();
    
    RobotProperties get_robot_properties () const throw ();
    
    void set_drive_vector (DriveVector dv) throw (BadHardwareException, HardwareException);

    void get_odometry() throw (BadHardwareException, HardwareException);
    
  protected:
    RobotProperties robot_properties;
    OdeSimPackClient* client;

  };
}

#endif

