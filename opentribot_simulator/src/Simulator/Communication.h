
#ifndef _TribotsSim_Communication_h_
#define _TribotsSim_Communication_h_

#include "../Communication/SimulatorUDPCommunication.h"
#include "../Fundamental/Time.h"
#include <vector>
#include "World.h"
#include <opentribot_messages/TribotDrive.h>
#include <opentribot_messages/WorldModel.h>
#include <ros/ros.h>

namespace TribotsSim {

  /** Komminaktionsklasse auf Serverseite */
  class Communication {
  public:
    Communication ();
    ~Communication ();

    void communicate (World&);
    void set_drive_command(const opentribot_messages::TribotDrive::ConstPtr & msg);    
    void  send_worldmodel_ros();
  private:


    ros::Publisher worldmodel_publisher;

    SimulatorUDPCommunication comm;
    std::vector<bool> partner_active;
    std::vector<struct sockaddr_in> partner_addresses;
    std::vector<Tribots::Time> partner_timeout;
  };

}

#endif
