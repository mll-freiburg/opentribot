//#include <stdarg.h>
//#include <stdio.h>
//#include <stdlib.h>


#include <cwiid.h>


#include <cmath>

//include <sstream>
//#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "opentribot_messages/TribotDrive.h"
#include "opentribot_messages/TeamSignal.h"
#include "opentribot_messages/RefSignal.h"
#include "opentribot_messages/VisionSignal.h"
//#include "opentribot_decision/src/Structures/GameState.h"
#include <iostream>
#include <ostream>




 /** Liste der moeglichen Refereebox-Signale */
  enum TeamSignal {
    TeamSIGnop,                  ///< ohne Bedeutung, veraendert nichts
    TeamSIGstop,                  ///< ohne Bedeutung, veraendert nichts
    TeamSIGstayaway,
    TeamSIGgivePass,
    TeamSIGgoForIt,
    TeamSIGhelpDefend,
    TeamSIGreceivePass,
  };


  /** Struktur, um die momentane Spielphase zu erfassen */
  struct Team {
    TeamSignal teamstate;///< aktueller Refereestate
    double posx;
    double posy;

  };




  static const int num_team_signals = 8;                  ///< Anzahl moeglicher Signale
  static const char teamsignal_names [8][23] = {         ///< Bezeichner (strings) der verschiedenen Signale
    "TeamSIGnop          ",
    "TeamSIGstop         ",
    "TeamSIGstayAway     ",
    "TeamSIGgivePass     ",
    "TeamSIGgoForIt      ",
    "TeamSIGhelpDefend   ",
    "TeamSIGreceivePass  ",
 };



 enum RefboxSignal {
    SIGnop,                  ///< ohne Bedeutung, veraendert nichts 
    SIGstop,
    SIGhalt,
    SIGstart,
    SIGready,
    SIGcyanKickOff,
    SIGmagentaKickOff,
    SIGownKickOff,
    SIGopponentKickOff,
    SIGcyanFreeKick,
    SIGmagentaFreeKick,
    SIGownFreeKick,
    SIGopponentFreeKick,
    SIGcyanGoalKick,
    SIGmagentaGoalKick,
    SIGownGoalKick,
    SIGopponentGoalKick,
    SIGcyanCornerKick,
    SIGmagentaCornerKick,
    SIGownCornerKick,
    SIGopponentCornerKick,
    SIGcyanThrowIn,
    SIGmagentaThrowIn,
    SIGownThrowIn,
    SIGopponentThrowIn,
    SIGcyanPenalty,
    SIGmagentaPenalty,
    SIGownPenalty,
    SIGopponentPenalty,
    SIGcyanGoalScored,
    SIGmagentaGoalScored,
    SIGownGoalScored,
    SIGopponentGoalScored,
    SIGdroppedBall,
    SIGtest1,
    SIGtest2,
    SIGtest3,
    SIGtest4,
    SIGtest5,
    SIGtest6,
    SIGtest7,
    SIGtest8,
  };


using namespace std;

/* This is a sample program written to demonstrate basic CWiid libwiimote
 * usage, until _actual_ documentation can be written.  It's quick and dirty
 * has a horrible interface, but it's sparce enough to pick out the important
 * parts easily.  For examples of read and write code, see wmgui.  Speaker
 * support is "experimental" (read: bad) enough to be disabled.  The beginnings
 * of a speaker output function are in libwiimote source. */
/* Note: accelerometer (including nunchuk) and IR outputs produce a
 * lot of data - the purpose of this program is demonstration, not good
 * interface, and it shows. */

cwiid_mesg_callback_t cwiid_callback;

int kick_charge=0;
int cyclecounter=0;

// Global exit flag
int exitflag = 0;
// Global active flag
int steering_active=0;
int stopcounter=10;
  int msgcount = 0;
	double x,y,rot;
int kick;
int klength;
  ros::Publisher *tribotdrive_publisher;
  ros::Publisher *refsignal_publisher;
  ros::Publisher *teamsignal_publisher;
  ros::Publisher *visionsignal_publisher;

  opentribot_messages::RefSignal refsignalmessage;
  opentribot_messages::TeamSignal teamsignalmessage;
  opentribot_messages::TeamSignal visionsignalmessage;

#define toggle_bit(bf,b)	\
	(bf) = ((bf) & b)		\
	       ? ((bf) & ~(b))	\
	       : ((bf) | (b))

#define MENU \
	"1: toggle LED 1\n" \
	"2: toggle LED 2\n" \
	"3: toggle LED 3\n" \
	"4: toggle LED 4\n" \
	"5: toggle rumble\n" \
	"a: toggle accelerometer reporting\n" \
	"b: toggle button reporting\n" \
	"c: enable motionplus, if connected\n" \
	"e: toggle extension reporting\n" \
	"i: toggle ir reporting\n" \
	"m: toggle messages\n" \
	"p: print this menu\n" \
	"r: request status message ((t) enables callback output)\n" \
	"s: print current state\n" \
	"t: toggle status reporting\n" \
	"x: exit\n"

void set_led_state(cwiid_wiimote_t *wiimote, unsigned char led_state);
void set_rpt_mode(cwiid_wiimote_t *wiimote, unsigned char rpt_mode);
void print_state(struct cwiid_state *state);

cwiid_err_t err;
void err(cwiid_wiimote_t *wiimote, const char *s, va_list ap)
{
	if (wiimote) printf("%d:", cwiid_get_id(wiimote));
	else printf("-1:");
	vprintf(s, ap);
	printf("\n");
}

int rumble(cwiid_wiimote_t*wiimote,int ms){
	if (cwiid_set_rumble(wiimote,1)) {
	fprintf(stderr, "Error setting rumble\n");
	}
	usleep(ms*1000);
	if (cwiid_set_rumble(wiimote, 0)) {
	fprintf(stderr, "Error setting rumble\n");
	}

return 0;
}
void set_led_state(cwiid_wiimote_t *wiimote, unsigned char led_state)
{
	if (cwiid_set_led(wiimote, led_state)) {
		fprintf(stderr, "Error setting LEDs \n");
	}
}
	
void set_rpt_mode(cwiid_wiimote_t *wiimote, unsigned char rpt_mode)
{
	if (cwiid_set_rpt_mode(wiimote, rpt_mode)) {
		fprintf(stderr, "Error setting report mode\n");
	}
}



/* Prototype cwiid_callback with cwiid_callback_t, define it with the actual
 * type - this will cause a compile error (rather than some undefined bizarre
 * behavior) if cwiid_callback_t changes */
/* cwiid_mesg_callback_t has undergone a few changes lately, hopefully this
 * will be the last.  Some programs need to know which messages were received
 * simultaneously (e.g. for correlating accelerometer and IR data), and the
 * sequence number mechanism used previously proved cumbersome, so we just
 * pass an array of messages, all of which were received at the same time.
 * The id is to distinguish between multiple wiimotes using the same callback.
 * */
void cwiid_callback(cwiid_wiimote_t *wiimote, int mesg_count,
                    union cwiid_mesg mesg[], struct timespec *timestamp)
{





	int i, j;

	x=0;
	y=0;
	rot=0;
	int valid_source;

	for (i=0; i < mesg_count; i++)
	{


		switch (mesg[i].type) {
		case CWIID_MESG_STATUS:
			printf("Status Report: battery=%d extension=",
			       mesg[i].status_mesg.battery);
			switch (mesg[i].status_mesg.ext_type) {
			case CWIID_EXT_NONE:
				printf("none");
				break;
			case CWIID_EXT_NUNCHUK:
				printf("Nunchuk");
				break;
			case CWIID_EXT_CLASSIC:
				printf("Classic Controller");
				break;
			case CWIID_EXT_BALANCE:
				printf("Balance Board");
				break;
			case CWIID_EXT_MOTIONPLUS:
				printf("MotionPlus");
				break;
			default:
				printf("Unknown Extension");
				break;
			}
			printf("\n");
			break;
		case CWIID_MESG_BTN:
			printf("Button Report: %.4X\n", mesg[i].btn_mesg.buttons);
			if (mesg[i].btn_mesg.buttons & CWIID_BTN_A){
			cout << "START"<<endl;
    			//refsignalmessage.refsig=SIGstart;refsignal_publisher->publish(refsignalmessage);
				teamsignalmessage.teamsignal=TeamSIGnop;teamsignal_publisher->publish(teamsignalmessage);
			}
			
			if (mesg[i].btn_mesg.buttons & CWIID_BTN_B){
			cout << "STOP"<<endl;
    			//refsignalmessage.refsig=SIGstop;refsignal_publisher->publish(refsignalmessage);
    			teamsignalmessage.teamsignal=TeamSIGstop;
    			cout <<"Teamsignal :"<<TeamSIGstop<<endl;teamsignal_publisher->publish(teamsignalmessage);
			}
			if (mesg[i].btn_mesg.buttons & CWIID_BTN_1){
				refsignalmessage.refsig=SIGstart;refsignal_publisher->publish(refsignalmessage);
			}
			if (mesg[i].btn_mesg.buttons & CWIID_BTN_2){
				refsignalmessage.refsig=SIGownKickOff;refsignal_publisher->publish(refsignalmessage);
			}
			if (mesg[i].btn_mesg.buttons & CWIID_BTN_MINUS){
			    teamsignalmessage.teamsignal=TeamSIGgivePass;teamsignal_publisher->publish(teamsignalmessage);
			    cout <<"Teamsignalmessage: "<<teamsignal_names[teamsignalmessage.teamsignal]<<endl;
			}
			if (mesg[i].btn_mesg.buttons & CWIID_BTN_PLUS){
				teamsignalmessage.teamsignal=TeamSIGreceivePass;teamsignal_publisher->publish(teamsignalmessage);
				cout <<"Teamsignalmessage: "<<teamsignal_names[teamsignalmessage.teamsignal]<<endl;
			}

			  
			  
			  
			
			
			
			break;
		case CWIID_MESG_ACC:
			printf("Acc Report: x=%d, y=%d, z=%d\n",
                   mesg[i].acc_mesg.acc[CWIID_X],
			       mesg[i].acc_mesg.acc[CWIID_Y],
			       mesg[i].acc_mesg.acc[CWIID_Z]);
			break;
		case CWIID_MESG_IR:
			printf("IR Report: ");
			valid_source = 0;
			for (j = 0; j < CWIID_IR_SRC_COUNT; j++) {
				if (mesg[i].ir_mesg.src[j].valid) {
					valid_source = 1;
					printf("(%d,%d) ", mesg[i].ir_mesg.src[j].pos[CWIID_X],
					                   mesg[i].ir_mesg.src[j].pos[CWIID_Y]);
				}
			}
			if (!valid_source) {
				printf("no sources detected");
			}
			printf("\n");
			break;
		case CWIID_MESG_NUNCHUK:
			
		/*	printf("\nNunchuk Report: btns=%.2X stick=(%d,%d) acc.x=%d acc.y=%d "
		       "acc.z=%d\n", mesg[i].nunchuk_mesg.buttons,
			       mesg[i].nunchuk_mesg.stick[CWIID_X],
			       mesg[i].nunchuk_mesg.stick[CWIID_Y],
			       mesg[i].nunchuk_mesg.acc[CWIID_X],
			       mesg[i].nunchuk_mesg.acc[CWIID_Y],
			       mesg[i].nunchuk_mesg.acc[CWIID_Z]);*/
			break;
		case CWIID_MESG_CLASSIC:
			printf("Classic Report: btns=%.4X l_stick=(%d,%d) r_stick=(%d,%d) "
			       "l=%d r=%d\n", mesg[i].classic_mesg.buttons,
			       mesg[i].classic_mesg.l_stick[CWIID_X],
			       mesg[i].classic_mesg.l_stick[CWIID_Y],
			       mesg[i].classic_mesg.r_stick[CWIID_X],
			       mesg[i].classic_mesg.r_stick[CWIID_Y],
			       mesg[i].classic_mesg.l, mesg[i].classic_mesg.r);
			break;
		case CWIID_MESG_BALANCE:
			printf("Balance Report: right_top=%d right_bottom=%d "
			       "left_top=%d left_bottom=%d\n",
			       mesg[i].balance_mesg.right_top,
			       mesg[i].balance_mesg.right_bottom,
			       mesg[i].balance_mesg.left_top,
			       mesg[i].balance_mesg.left_bottom);
			break;
		case CWIID_MESG_MOTIONPLUS:
			printf("MotionPlus Report: angle_rate=(%d,%d,%d)\n",
			       mesg[i].motionplus_mesg.angle_rate[0],
			       mesg[i].motionplus_mesg.angle_rate[1],
			       mesg[i].motionplus_mesg.angle_rate[2]);
			break;
		case CWIID_MESG_ERROR:
			if (cwiid_close(wiimote)) {
				fprintf(stderr, "Error on wiimote disconnect\n");
				exit(-1);
			}
		exitflag=1;	
		cout <<"CWIID_MESG_ERROR CASE"<<endl;
			break;
		default:
			printf("Unknown Report");
			break;
		}


/*rot = (mesg[i].motionplus_mesg.angle_rate[2]-8300)/500.0;
   cout << "Sending stuff"<<endl;
   cout << "mesg "<<mesg[i].btn_mesg.buttons<<endl;

*/

kick=false;
klength=0;

struct cwiid_state state;	/* wiimote state */
if (cwiid_get_state(wiimote, &state)) {
                                fprintf(stderr, "Error getting state\n");
                        }

rot=(state.ext.nunchuk.stick[CWIID_X]-129)*-3.0/128;
y=(state.ext.nunchuk.stick[CWIID_Y]-126)*2.0/128;
if (fabs(y)<0.2)y=0;
/*if(state.buttons & CWIID_BTN_UP)y=3.0;
if(state.buttons & CWIID_BTN_DOWN)y=-3.0;
*/
//cout << state.ext.motionplus.angle_rate[0]<<endl;
//cout << state.ext.motionplus.angle_rate[1]<<endl;
//cout << state.ext.motionplus.angle_rate[2]<<endl;
//rot=(state.ext.nunchuk.acc[CWIID_X]-128)*(-1)*20.0/128;
if(state.buttons & CWIID_BTN_LEFT)x-=2.0;
if(state.buttons & CWIID_BTN_RIGHT)x+=2.0;
if(state.buttons & CWIID_BTN_DOWN)y-=2.0;
if(state.buttons & CWIID_BTN_UP)y+=2.0;


if (state.ext.nunchuk.buttons & CWIID_NUNCHUK_BTN_Z){
kick_charge+=1;
cout <<"CHArging ....................!!! "<<endl;
}

else {

	if (kick_charge>0){

		kick=true;
		klength= kick_charge*10;
		if (klength> 400) klength=400;
		cout <<"sending kick with klength="<<klength<<endl;
		cout <<"sending kick with klength="<<klength<<endl;
		cout <<"sending kick with klength="<<klength<<endl;

}
		kick_charge=0;

}



cyclecounter++;
//if(cyclecounter%600==0)	cwiid_set_rumble(wiimote,1);
//if(cyclecounter%602==0)	cwiid_set_rumble(wiimote,0);

/*if (cyclecounter%200==0) cwiid_set_rumble(wiimote,1);
if (cyclecounter%202==0) cwiid_set_rumble(wiimote,0);
*/
//cout << "ACC="<<(int )state.ext.nunchuk.acc[CWIID_X];

//if (cyclecounter==2000)cwiid_disable(wiimote,CWIID_FLAG_MESG_IFC);

if (state.buttons & CWIID_BTN_HOME){
	// kill the program !!!

        exitflag=1;
//	ros::shutdown();

}

opentribot_messages::TribotDrive msg;

if (state.buttons &CWIID_BTN_B){// BUTTON B is "DEAD MAN SWITCH !!"
if (steering_active==0){


}

if (CWIID_MESG_STATUS==CWIID_MESG_ERROR) cout << "CWIID_MESG_ERROR"<<endl;

steering_active=1;
stopcounter=0;


    std::stringstream ss;
    ss << "WIIMOTE" << msgcount;
    ++msgcount;
    msg.sender=ss.str();
    cout <<"."<<flush;    
    msg.vtransx=x;
    msg.vtransy=y;
    msg.vrot=rot;
    msg.kick=kick;
    msg.kicklength=klength;
	
    ROS_INFO("%s %f %f", msg.sender.c_str(), msg.vtransx, msg.vtransy );

    tribotdrive_publisher->publish(msg);



}
else{
stopcounter++;

  steering_active=0;
  x=0;
  y=0;
  rot=0;
  kick=0;
  msg.vtransx=x;
  msg.vtransy=y;
  msg.vrot=rot;
  msg.kick=kick;
if (stopcounter<5){tribotdrive_publisher->publish(msg);
}


}





	}
}


//
//
//
//
//
//

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher pub1 = n.advertise<opentribot_messages::TribotDrive>("drivecommands", 100);
  ros::Publisher pub2 = n.advertise<opentribot_messages::TeamSignal>("teamsignals", 100);
  ros::Publisher pub3 = n.advertise<opentribot_messages::RefSignal>("refsignals", 100);
  ros::Publisher pub4 = n.advertise<opentribot_messages::VisionSignal>("VisionSignal",100);
  tribotdrive_publisher=&pub1;
  teamsignal_publisher=&pub2;
  refsignal_publisher=&pub3; 
  visionsignal_publisher=&pub4;
  std::string bluetooth_address;
  double speed;
  if (!n.getParam("/wii_remote/bluetooth_address", bluetooth_address))
  {
  	n.setParam("/wii_remote/bluetooth_address","ANY");
	ROS_INFO("Konnte parameter nicht auslesen");
  }

  if (!n.getParam("/wii_remote/Speed", speed))
  {
	n.setParam("/wii_remote/Speed",2);
	ROS_INFO("Konnte parameter nicht auslesen");
  }
  ROS_INFO("WII-Geschwindigkeit %f",speed);
   // Default value version
  cwiid_wiimote_t *wiimote;	/* wiimote handle */
  struct cwiid_state state;	/* wiimote state */
  bdaddr_t bdaddr;	/* bluetooth device address */
  unsigned char mesg = 0;
  unsigned char led_state = 0;
  unsigned char rpt_mode = 0;
  unsigned char rumble = 0;
  
  cwiid_set_err(err);
	/* Connect to address given on command-line, if present */
/*	if (argc > 1) {
		str2ba(argv[1], &bdaddr);
	}
	else {
		bdaddr = *BDADDR_ANY;
	}
*/


//		bdaddr = *BDADDR_ANY;

char*wiimote1;
 

//if (strcmp(argv[3],"ANY")==0){
if (strcmp(bluetooth_address.c_str(),"ANY")==0){
//ROS_INFO( "using ANY wiimote");
bdaddr=*BDADDR_ANY;
}
else{ 
wiimote1=(char*)bluetooth_address.c_str();
str2ba(wiimote1,&bdaddr);


}

//#define  BDADDR_WIIMOTE1  (&(bdaddr_t){{0x00,0x21,0xbd,0x01,0x3E,0xEE}}) 
//bdaddr=*BDADDR_WIIMOTE1;
	/* Connect to the wiimote */

//while(true){
while (true){
	
    	if (!ros::ok())exit(-1);
	ROS_INFO("Put Wiimote in discoverable mode now (press 1+2)...\n");
	if (!(wiimote = cwiid_open(&bdaddr, 0))) {
		ROS_ERROR("Unable to connect to wiimote\n");
		continue;
	}
	if (cwiid_set_mesg_callback(wiimote, cwiid_callback)) {
		ROS_ERROR("Unable to set message callback\n");
	}

	set_led_state(wiimote, CWIID_LED1_ON|CWIID_LED2_ON|CWIID_LED3_ON|CWIID_LED4_ON);
        cwiid_enable(wiimote, CWIID_FLAG_MOTIONPLUS);
	//toggle_bit(rpt_mode, CWIID_RPT_EXT);
	set_rpt_mode(wiimote, CWIID_RPT_EXT|CWIID_RPT_BTN);
	//toggle_bit(rpt_mode, CWIID_RPT_BTN);
	//set_rpt_mode(wiimote, rpt_mode);
	for (int j=0;j<5;j++){ // RUMBLING A LITTLE TO MAKE SOME FEEDBACK
	toggle_bit(rumble, 1);
	if (cwiid_set_rumble(wiimote, rumble)) {
	fprintf(stderr, "Error setting rumble\n");
	}
	usleep(j*20000);
	toggle_bit(rumble, 1);
	if (cwiid_set_rumble(wiimote, rumble)) {
	fprintf(stderr, "Error setting rumble\n");
	}
	usleep(j*20000);
	}

	if (cwiid_enable(wiimote, CWIID_FLAG_MESG_IFC)) {
		fprintf(stderr, "Error enabling messages\n");
	}
	

        /* Menu */
	printf("%s", MENU);

	exitflag=0;
	
        ros::Rate loop_rate(10);
        while (!exitflag)
	{
    	  if (!ros::ok())exit(-1);
          ros::spinOnce();
  	  loop_rate.sleep();
	}

/*	if (cwiid_close(wiimote)) {
		fprintf(stderr, "Error on wiimote disconnect\n");
		return -1;
	}
*/

}	

//}
	return 0;
}

