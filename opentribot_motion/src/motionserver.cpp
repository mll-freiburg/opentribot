
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "opentribot_messages/TribotDrive.h"
#include "motionserver.h"
//#include <../../opt/ros/diamondback/ros/core/roscpp/include/ros/forwards.h>
#include <termios.h>

using namespace std;
/**
 *  * This tutorial demonstrates simple receipt of messages over the ROS system.
 *   */
struct DriveVector{
double x,y,vrot;
int kick;
int klength;
};
        enum {NOKICK=0, HIGHKICK=1, LOWKICK=2};



  DriveVector localdv;

  Tribots::Time last_command;




void kick(int kicker,int kicklength){
     
     if (kicker==HIGHKICK)
	usbdevice.kick(0,kicklength);
     if (kicker==LOWKICK)
	usbdevice.kick(1,30);

//	cout <<"KICKING"<<endl;


    }





void receiveDriveMessageCallback(const opentribot_messages::TribotDrive::ConstPtr& msg)
{
  //ROS_INFO("Motionserver heard: [%s]", msg->vtransx,msg->vtransy,msg->vrot);


    last_command.update();


    localdv.x=msg->vtransx;
    localdv.y=msg->vtransy;
    localdv.vrot=msg->vrot;
    localdv.kick=msg->kick;
    localdv.klength=msg->kicklength;
    if (localdv.kick !=NOKICK)kick(localdv.kick,localdv.klength);




}








int main(int argc, char **argv)
{
/***********************************ROS**************************/
  ros::init(argc, argv, "motionserver");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("drivecommands", 10, receiveDriveMessageCallback);
  ros::Rate loop_rate(50);
/***********************************ENDROS***********************/


  last_command.set_sec(-1000);  
  bool roboter_an=false; 

  //fuer select 
  fd_set rfds;
  timeval tv;
  
 bool manual_off=false;

  usbdevice.init();
  //imu.usbdevice=&usbdevice;
  //imu.loadcalibration();
  //imu.debug=false;
//  signal (SIGINT, breakup_handler);
//  signal (SIGSEGV, breakup_handler);
//  signal (SIGKILL, breakup_handler);


  /*serversocket = new UDPSocket ();
  serversocket->init_as_server (51111);
*/
  localdv.x = 0;
  localdv.y = 0;
  localdv.vrot = 0;

  float vel[3];
  vel[0] = 0;
  vel[1] = 0;
  vel[2] = 0;

  ramp motionRampsTrans (200);
  ramp motionRampsRot (200);

  if (argc != 4) {
    localdv.x = 0;
    localdv.y = 0;
    localdv.vrot = 0;
  } else {
    localdv.x = atof (argv[1]);
    localdv.y = atof (argv[2]);
    localdv.vrot = atof (argv[3]);

  }



  EposStruct epos[3];
/*
  epos[0].init (2);
  epos[1].init (4);
  epos[2].init (8);



*/
#ifdef USE_HARDWARE
  can.initializeCan ();

#endif



  Tribots::Time current_time;
  int time_10framesago;

  int loopcounter = 0;
  int framecounter = 0;
  DriveVector odo;




int count = 0;
while (ros::ok())
{

//roboter_an=true;

  if (roboter_an==true && last_command.elapsed_sec()>30){
	printf("last_command sec %i \n",last_command.elapsed_sec());
    roboter_an=false;
    usbdevice.power_down();


    }
    if (roboter_an==false && last_command.elapsed_sec()<30){
	printf("last_command sec %i \n",last_command.elapsed_sec());

	usbdevice.power_up();
    
    sleep(1);
    epos[0].init (2);
    	epos[1].init (4);
    	epos[2].init (8);

    roboter_an=true;

    }



/*    epos[0].check_web_inputs();
    epos[0].update_parameters();
   epos[1].check_web_inputs();
  epos[1].update_parameters();
    epos[2].check_web_inputs();
    epos[2].update_parameters();
*/
    current_time.update ();
    framecounter++;
    double fps;
   if (framecounter % 60 == 0) {
      fps = 1000 * 60.0f / (current_time.get_msec () - time_10framesago);
      cout << "FPS" << fps << endl;
      time_10framesago = current_time.get_msec ();
    }
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    int stdin_fd;
    //int max_fd_num = serversocket->socketDescriptor;
    FD_ZERO (&rfds);
    stdin_fd = fileno (stdin);
    int max_fd_num = stdin_fd;
    FD_SET (stdin_fd, &rfds);
//    FD_SET (serversocket->socketDescriptor, &rfds);

    char c = (-1);

    struct termios to, t;
    tcgetattr (1, &t);
    // saving termios struct
    to = t;

    //toggling canonical mode and echo mode 
    t.c_lflag &= ~(ICANON);
    tcsetattr (1, 0, &t);

    int retval = select (max_fd_num + 1, &rfds, NULL, NULL, &tv);

    if (FD_ISSET (stdin_fd, &rfds))	// Tastatureingabe;
    {
      last_command.update();

      char eingabe[100];

      ssize_t r_bytes;		// number of bytes read into the buffer
      c = getchar ();

      if (c == 'w')
	localdv.y += 0.2f;
      if (c == 'v')
	debug = !debug;
      if (c == 's')
	localdv.y -= 0.2f;
      if (c == 'd')
	localdv.x += 0.2f;
      if (c == 'a')
	localdv.x -= 0.2f;
      if (c == ' ')
	localdv.y = 0.0f;
      if (c == ' ')
	localdv.x = 0.0f;
      if (c == ' ')
	localdv.vrot = 0.0f;
      if (c == 'j')
	localdv.vrot += 0.2f;
      if (c == 'k')
	localdv.vrot = 0.0f;
      if (c == 'l')
	localdv.vrot -= 0.2f;
      if (c == 'q')
	ros::shutdown();
      if (c == 'i') {
	can.initializeCan ();
	epos[0].init (2);
	epos[1].init (4);
	epos[2].init (8);
      }
      if (c == 'P') {
	kick(HIGHKICK,50);
      }
      if (c == 'O') {
	kick(LOWKICK,50);
      } 
      if (c == 'o') {
	cout << "Switching off Power to Motors..."<<endl;
	  usbdevice.power_down();
	manual_off=true;
    roboter_an=false;
      }

      if (localdv.x > 3)
	localdv.x = 3;
      if (localdv.x < -3)
	localdv.x = -3;
      if (localdv.y > 3)
	localdv.y = 3;
      if (localdv.y < -3)
	localdv.y = -3;
      if (localdv.vrot > 6)
	localdv.vrot = 6;
      if (localdv.vrot < -6)
	localdv.vrot = -6;

      loopcounter = 0;
      cout << "LOOPCOUNTER KEYBOARD=0" << endl;
    }

tcsetattr (1, 0, &to);


// kick 
//



//    imu.imu_get_values();

    double sollrot=localdv.vrot;
//    double istrot=-imu.yaw*M_PI/180;
//    cout <<"sollrot_:"<<sollrot<<" ISTrot  "<<istrot <<endl;
	

  //  if (fabs(istrot)<0.1f)istrot=0;
  //  double imurot=sollrot+0.4f*(sollrot-istrot);

    //imurot=localdv.vrot;	




    loopcounter++;

    if (loopcounter > 60) {
	if (loopcounter%10==0)      cout << "S";
    //  usleep(15000);
      localdv.x *= 0.9f;
      localdv.y *= 0.9f;
      localdv.vrot *= 0.9f;
      if (fabs (localdv.x) < 0.1)
	localdv.x = 0;
      if (fabs (localdv.y) < 0.1)
	localdv.y = 0;
      if (fabs (localdv.vrot) < 0.1)
	localdv.vrot = 0;

    }
    //usleep (10000);


/*
      cout << "X=" << setprecision (1) << setw (10) << setfill (' ') <<
	localdv.
	x << "   Y=" << setprecision (1) << setw (10) << setfill (' ') <<
	localdv.
	y << "  PHI=" << setprecision (1) << setw (10) << setfill (' ') <<
	localdv.vrot;

*/

    int speed1 =500;
    int speed2 =500;
    int speed3 =500;
    int setvel =0;
//int accel_step = 103012;
int accel_step = 13012;
    int max_vrot =344;
    int accel_step_rot =12727;

    //set maximal accelerations for rotation and translation
    motionRampsTrans.setMaxchangeps (accel_step);
    motionRampsRot.setMaxchangeps (accel_step_rot);

    float max_vrot_float=max_vrot*1.0f/100;

     if (localdv.vrot>max_vrot_float) localdv.vrot=max_vrot_float;
     if (localdv.vrot<-max_vrot_float) localdv.vrot=-max_vrot_float;


    // idee:   zuerst vels fuer translational berechnen, danach rotational und dann 2 unterschiedliche accelerations 
    // zuerst translational berechnen
    robot_comp_thetap_of_wvel (localdv.x, localdv.y, 0, 0,
			       &vel[0], &vel[1], &vel[2]);

    // rad/s auf upm
    // Gear-Ratio beruecksichtigen
    
    double gear_factor=(60*gear_ratio)/(2*M_PI);
    
    
    
    for (int i = 0; i < 3; i++) {
      vel[i] = vel[i] * gear_factor;
    }

    // MOTION SMOOTHING 
    motionRampsTrans.update (vel, 3);

    float velrot[3];
    ////////////  KINEMATIK Rotational
    robot_comp_thetap_of_wvel (0, 0, localdv.vrot, 0, &velrot[0],
   //robot_comp_thetap_of_wvel (0, 0, imurot, 0, &velrot[0],
			       &velrot[1], &velrot[2]);

    // rad/s auf upm
    // Gear-Ratio beruecksichtigen
    for (int i = 0; i < 3; i++) {

      velrot[i] = velrot[i] * gear_factor;
    }


    // MOTION SMOOTHING 
    motionRampsRot.update (velrot, 3);


    ////////// Combine Translational and Rotatinoal components
    for (int i = 0; i < 3; i++) {
      vel[i] = vel[i] + velrot[i];
    }

    //motor velocities are known according to acceleration model
    //calculate back the wheel velocisites as a hint for the world model

    float wheelspeed_sent[3];
    float x, y, phi;
    for (int i = 0; i < 3; i++) {
      wheelspeed_sent[i] = vel[i] /gear_factor;
    }
    robot_comp_mvel_of_thetap (wheelspeed_sent[0], wheelspeed_sent[1],
			       wheelspeed_sent[2], &x, &y, &phi);


#ifdef  USE_HARDWARE

    //     cerr << "Zyklus vor update *****************************************" << endl;

    if (roboter_an){

    int all_ok = true;
    all_ok &= epos[0].update (-vel[2]);
    all_ok &= epos[1].update (-vel[0]);
    all_ok &= epos[2].update (-vel[1]);

    float odoradvel[3];
    float odovel[3];
   


    odoradvel[2]=-epos[0].actualvel/gear_factor;
    odoradvel[0]=-epos[1].actualvel/gear_factor;
    odoradvel[1]=-epos[2].actualvel/gear_factor;
 
    // Umrechnen odoradvel nach und rot

    

    robot_comp_mvel_of_thetap (odoradvel[0], odoradvel[1],odoradvel[2],
				&odovel[0],&odovel[1],&odovel[2]);
    odo.x=odovel[0];
    odo.y=odovel[1];
    odo.vrot=odovel[2];
//    odo.vrot=istrot;   // Gyro


//	cout << "SET VELS : "<<localdv<<endl;
//	cout << "ODO VELS : "<<odo<<endl;



//    cout << "setvel="<<-vel[2]<<"getvel="<<epos[0].actualvel<<endl;

    if (!all_ok) {

      cout << "Fehler!! Ein Epos hat ein Problem " << endl;

      epos[0].resetflag=true;
      epos[1].resetflag=true;
      epos[2].resetflag=true;

      epos[0].update (0);
      epos[1].update (0);
      epos[2].update (0);
    }




    //if (epos[0].bootcounter + epos[1].bootcounter + epos[2].bootcounter > 20) {
    if (epos[0].bootcounter  > 20) {
      cerr << "resetting..." << endl;
      usleep(500000);
      can.initializeCan ();
      epos[0].bootcounter = 0;
      epos[1].bootcounter = 0;
      epos[2].bootcounter = 0;
    }


    }
#endif


ros::spinOnce();
loop_rate.sleep();
++count;
}

      epos[0].update (0);
      epos[1].update (0);
      epos[2].update (0);
	usbdevice.power_down();

return 0;
}
