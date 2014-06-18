
#include <cmath>
#include "JoystickPlayerUDP.h"
#include "PlayerFactory.h"
#include "../Structures/Journal.h"
#include "../WorldModel/WorldModel.h"
using namespace Tribots;
using namespace std;


// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (string("JoystickPlayerUDP"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, PlayerType*) throw (TribotsException,bad_alloc) {
      return new JoystickPlayerUDP (reader);
    }
  };
  Builder the_builder;
}





namespace {
  inline double square (const double& x) {
    return x*x;
  }
}

JoystickPlayerUDP::JoystickPlayerUDP (const ConfigReader& vread) throw (InvalidConfigurationException, std::bad_alloc)  {
  // default-Werte fuer Joystick-Achsen
  x_axis = 0;
  y_axis = 1;
  phi_axis = 3;
  x_diraxis = +1;
  y_diraxis = -1;
  phi_diraxis = -1;
  kick_button = 2;
  accel_button = 9;
  decel_button = 8;

  max_velocity = 0.5*WorldModel::get_main_world_model().get_robot_properties().max_velocity;
  max_rot_velocity = 0.5*WorldModel::get_main_world_model().get_robot_properties().max_rotational_velocity;
  previously_changed=true;

  vector<int> ii (3);
  if (vread.get ("Joystick::axis", ii)>=3) {
    x_axis = ii[0];
    y_axis = ii[1];
    phi_axis = ii[2];
  }
  if (vread.get ("Joystick::diraxis", ii)>=3) {
    x_diraxis = ii[0];
    y_diraxis = ii[1];
    phi_diraxis = ii[2];
  }
  if (vread.get ("Joystick::buttons", ii)>=3) {
    kick_button = ii[0];
    accel_button = ii[2];
    decel_button = ii[1];
  }
 serversocket=new UDPSocket();
 serversocket->init_as_server(51111);
}

JoystickPlayerUDP::~JoystickPlayerUDP () throw () {
}

DriveVector JoystickPlayerUDP::process_drive_vector (Time t) throw () {
//cout << "Axis:"<<js.axis.size()<<"whole struct"<<sizeof(struct Joystick)<<endl;


 unsigned int writtendata=0;
  serversocket->receive((char*)&jstate,writtendata,sizeof(JoystickState)); 
  DriveVector dv (Vec(0,0), 0, false);
 
  if (writtendata>0){
//cout << "Axis:"<<js.axis.size()<<"whole struct"<<sizeof(struct Joystick)<<endl;
 cout << "Received"<<writtendata<<" elementsendl"<<endl;
        double maxax=0;
        int whichmax=0;
    for (int i=0;i<10;i++){
                if (fabs(jstate.axis[i])>maxax)
                {
                        whichmax=i;
                        maxax=fabs(jstate.axis[i]);
                }
    }
//     cout <<"Axis "<<whichmax<<"has biggest value:"<< jstate.axis[whichmax] <<endl;
    
    
}
cout << "0 1 2 : "<<jstate.axis [0]<<" "<<jstate.axis[1]<<" "<<jstate.axis[2]<<endl;
dv.vtrans.x=jstate.axis[0]*3;
 dv.vtrans.y=-jstate.axis[1]*3;
 dv.vrot=-jstate.axis[2]*4;
double w[3];
w[0]=-jstate.axis[0]*30;
w[1]=jstate.axis[0]*30;
w[2]=jstate.axis[1]*30;
DriveVector dv2(w[0],w[1],w[2],false,WHEELVELOCITY);
dv.kick=jstate.buttons[2];

  return dv;
}

