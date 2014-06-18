#include "RobotData.h"
#include <stdio.h>
Tribots::RobotData::RobotData() throw ()
{
  sprintf(robotIdString,"NOT DEFINED");
  BoardID    = -1;
  motors_on  = true;
  
  for (int i=0; i<3; i++)
    {
      wheel_vel[i]         = 0;
      robot_vel[i]         = 0;
      motor_current[i]     = 0;
      motor_output[i]      = 0;
      motor_temp_switch[i] = 0;
      motor_temp[i]        = 0;
      wheel_vel_set[i]     = 0;
      robot_vel_set[i]     = 0;
    }
  motor_vcc = 0;
  dv_set    = DriveVector();
}

Tribots::RobotData::RobotData (const RobotData& src) throw ()
{
  sprintf(robotIdString,"%s", src.robotIdString);
  BoardID    = src.BoardID;
  motors_on  = src.motors_on;
  
  for (int i=0; i<3; i++)
    {
      wheel_vel[i]         = src.wheel_vel[i];
      robot_vel[i]         = src.robot_vel[i];
      motor_current[i]     = src.motor_current[i];
      motor_output[i]      = src.motor_output[i];
      motor_temp_switch[i] = src.motor_temp_switch[i];
      motor_temp[i]        = src.motor_temp[i];
      wheel_vel_set[i]     = src.wheel_vel_set[i];
      robot_vel_set[i]     = src.robot_vel_set[i];
    }
  motor_vcc = src.motor_vcc;
  dv_set = src.dv_set;
}

const Tribots::RobotData& Tribots::RobotData::operator= (const Tribots::RobotData& src) throw ()
{
  sprintf(robotIdString,"%s", src.robotIdString);
  BoardID    = src.BoardID;
  motors_on  = src.motors_on;
  
  for (int i=0; i<3; i++)
    {
      wheel_vel[i]         = src.wheel_vel[i];
      robot_vel[i]         = src.robot_vel[i];
      motor_current[i]     = src.motor_current[i];
      motor_output[i]      = src.motor_output[i];
      motor_temp_switch[i] = src.motor_temp_switch[i];
      motor_temp[i]        = src.motor_temp[i];
      wheel_vel_set[i]     = src.wheel_vel_set[i];
      robot_vel_set[i]     = src.robot_vel_set[i];
    }
  motor_vcc = src.motor_vcc;
  dv_set    = src.dv_set;

  return *this;
}

