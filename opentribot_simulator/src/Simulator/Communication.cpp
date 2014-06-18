
#include "Communication.h"
#include "global.h"
#include <cmath>

using namespace TribotsSim;
using namespace std;

Communication::Communication () {
	ros::NodeHandle n;
	worldmodel_publisher =n.advertise<opentribot_messages::WorldModel>("WorldModelMessages",1);
}

Communication::~Communication () {
}
 


void Communication::set_drive_command(const opentribot_messages::TribotDrive::ConstPtr & msg){

      global_world_pointer->robots[0].setSteeringX (msg->vtransy);
      global_world_pointer->robots[0].setSteeringY (-msg->vtransx);
      global_world_pointer->robots[0].setSteeringPhi (msg->vrot);
      global_world_pointer->robots[0].setSteeringKick (msg->kicklength>0);
      cout << "MSG KLENGTH"<<msg->kicklength<< " is >0 "<<(msg->kicklength>0)<<endl;


}

void Communication::send_worldmodel_ros(){
	const dReal* v;
   	opentribot_messages::WorldModel wmsg;
        double x=0,y=0,z=0,phi=0,vx=0,vy=0,vz=0,vphi=0;
      bool kick=false;
      	v=global_world_pointer->ball.getPosition();
	x=1000*v[1];
	y=-1000*v[0];
	z=1000*v[2];
	v=global_world_pointer->ball.getLinearVelocity();
	vx=v[1];
	vy=-v[0];
	vz=v[2];
   	wmsg.ball.pos.x=x;
	wmsg.ball.pos.y=y;
	wmsg.ball.pos.z=z;
	wmsg.ball.pos_known=true;
	wmsg.ball.velocity.x=vx;
	wmsg.ball.velocity.y=vy;
	wmsg.ball.velocity.z=vz;
	wmsg.ball.velocity_known=true;

	v = global_world_pointer->robots[0].getPosition();
      	x=1000*v[1];
      	y=-1000*v[0];
      	v = global_world_pointer->robots[0].getRotation();
      	phi=atan2 (-v[1], v[0])+M_PI;
      	v=global_world_pointer->robots[0].getLinearVelocity();
      	vx=v[1];
      	vy=-v[0];
      	v=global_world_pointer->robots[0].getAngularVelocity();
      	vphi=v[2];
      	kick=(fabs(global_world_pointer->robots[0].getSteering()[3])>0.001);
	wmsg.robot.pos.x=x;
	wmsg.robot.pos.y=y;
	wmsg.robot.kick=kick;
	wmsg.robot.heading.theta=phi;

	worldmodel_publisher.publish(wmsg);

}







 
void Communication::communicate (World& world) {


}
