#include "string.h"
//#include "bvserial.h
//
//
#include "bvcan.h"
#include "bvcanopenmessageset.h"
//#include "../../Structures/DriveVector.h"
//#include "../../Structures/DriveVectorReadWriter.h"
#include "Time.h"
#include "kicker/kick.h"
//#include "udp_imu/usbimu/imu.h"
#include "sstream"

#define BOOTUP                          0x0000
#define NOT_READY_TO_SWITCH_ON          0x0100
#define SWITCH_ON_DISABLED              0x0140
#define READY_TO_SWITCH_ON              0x0121
#define SWITCHED_ON                     0x0123
#define REFRESH                         0x4123
#define MEASURE_UNIT                    0x4133
#define OPERATION_ENABLED               0x0137
#define FAULT_REACTION_ACTIVE_DISABLED  0x0117
#define FAULT_REACTION_ACTIVE_ENABLED   0x011F
#define FAULT				0x0108

//knut
#define FAULT_OVER_CURRENT		0x120b




#define USE_HARDWARE

USBKickerImu usbdevice;
//Imu imu;


Tribots::Time last_sound;

int wait_time=0;
// Can-Device 
#ifdef USE_HARDWARE
BVCan can;
bool get_odometry=true;


#endif
// ROBOT SIZE DEFINITIONS AND PARAMETERS

#define TWOPI     2*M_PI
#define DELTA_DEF             30 * M_PI / 180.0
#define l1_def             0.22
#define l2_def             0.11
#define wheel_radius_def   0.06
#define gear_ratio	   22


#define dout if(debug)cout
/********************************************************************************************
 *  *  function to compute the velocity of the wheels (rad/s) based on the velocity in the world frame
 *   *  relative to an given world frame and the actual angle phi of the robot with the world frame
 *    *
 *     * use this function in a loop for small approximation steps,
 *      * angle phi will change in a continous way
 *       ********************************************************************************************/


bool debug = true;





int
robot_comp_thetap_of_wvel (float xposwp, float yposwp,
			   float phiwp, float phi, float *thetap1,
			   float *thetap2, float *thetap3) {
  float h1, h2;
  float R, L1, L2;
  float _delta_rad;
  L1 = l1_def;
  L2 = l2_def;
  R = wheel_radius_def;
  _delta_rad = DELTA_DEF;

  h1 = -(float) sin (double (_delta_rad + phi));
  h2 = (float) cos (double (_delta_rad + phi));
  *thetap1 = (h1 * xposwp + h2 * yposwp + L1 * phiwp) / R;

  h1 = -(float) sin (double (_delta_rad - phi));
  h2 = -(float) cos (double (_delta_rad - phi));
  *thetap2 = (h1 * xposwp + h2 * yposwp + L1 * phiwp) / R;

  h1 = (float) cos (double (phi));
  h2 = (float) sin (double (phi));
  *thetap3 = (h1 * xposwp + h2 * yposwp + L2 * phiwp) / R;

  return 0;
}

volatile int exitFlag = 0;


void
breakup_handler (int sig) {
  exitFlag = 1;			// im fehlerfall Exitflag setzen
}


/*th this function you can compute the resulting movement of the
 *  robot in the robots moving frame fixed on the robot.
 *   ************************************************************************************/
int
robot_comp_mvel_of_thetap (float thetap1, float thetap2,
			   float thetap3, float *xposmp,
			   float *yposmp, float *phimp) {
  /* computation of moving frame velocity  */
  float cosdelta, sindelta;
  float h1, h2, h3;
  float R, L1, L2;
  float _delta_rad;
  L1 = l1_def;
  L2 = l2_def;
  R = wheel_radius_def;
  _delta_rad = DELTA_DEF;

  cosdelta = (float) cos ((double) _delta_rad);
  sindelta = (float) sin ((double) _delta_rad);

  h1 = -((0.5 * L2) / (L1 + sindelta * L2)) * thetap1;
  h2 = -((0.5 * L2) / (L1 + sindelta * L2)) * thetap2;
  h3 = ((L1) / (L1 + sindelta * L2)) * thetap3;
  *xposmp = R * (h1 + h2 + h3);	// velocity in direction x in moving frame coords
if (fabs(*xposmp)<0.00001)*xposmp=0;

  h1 = (1 / (2 * cosdelta)) * thetap1;
  h2 = -(1 / (2 * cosdelta)) * thetap2;
  h3 = 0;
  *yposmp = R * (h1 + h2 + h3);
if (fabs(*yposmp)<0.00001)*yposmp=0;

  h1 = (1 / (2 * (L1 + sindelta * L2))) * thetap1;
  h2 = (1 / (2 * (L1 + sindelta * L2))) * thetap2;
  h3 = (sindelta / (L1 + sindelta * L2)) * thetap3;
  *phimp = R * (h1 + h2 + h3);
if (fabs(*phimp)<0.00001)*phimp=0;

  return 0;
}





struct ramp {
  float lastv[3];
  float maxchangeps;		//per second 

  Tribots::Time last_update;

    ramp (float maxchange_) {
    lastv[0] = 0;
    lastv[1] = 0;
    lastv[2] = 0;
    maxchangeps = maxchange_;
  }

  double setMaxchangeps (float n) {
    maxchangeps = n;
  }

  float update (float *v, int n) {

    int deltamsec = last_update.elapsed_msec ();
    last_update.update ();
    float biggest_change = 0;

    int maxchange = maxchangeps * deltamsec / 1000;



    for (int i = 0; i < n; i++) {
      float change;
      change = fabs (v[i] - lastv[i]);
      if (change > biggest_change) {
	biggest_change = change;
      }
    }


    //cout << "BIGGGEST                          CHANGE"<<biggest_change<<endl;

    if (biggest_change > maxchange) {
      for (int i = 0; i < n; i++) {
	// die änderung der geschwindigkeiten darf maximal maxchange betragen, soll aber laut urspruenglicher kinematik skaliert werden.
	v[i] = lastv[i] + (v[i] - lastv[i]) * (maxchange / biggest_change);
      }
    }


    for (int i = 0; i < n; i++) {
      lastv[i] = v[i];
    }

  }
};


int
read_can_answer (BVMessageTPCANMsg * msgReceive) {
  string strReceive;
#ifdef USE_HARDWARE
  can.read (&strReceive);
#endif
  msgReceive->stringToObject (strReceive);
  BV_DEBUGINFO4NNL ("RECEIVED: canID=" << hex << (int)
		    msgReceive->getCanID () << dec);
  BV_DEBUGINFO4C (" type=" << hex << (int) msgReceive->getMSGTYPE () << dec);
  BV_DEBUGINFO4C (" len=" << hex << (int) msgReceive->getLEN () << dec);
  BV_DEBUGINFO4C (" data=" << hex << "[" << (int) msgReceive->getDATA (0) <<
		  "]");
  for (int i = 1; i < 8; i++) {
    BV_DEBUGINFO4C ("[" << (int) msgReceive->getDATA (i) << "]");
  }
  BV_DEBUGINFO4C (dec << endl);

};

struct EposParameters{ 

 int      _read;
 int      _set;
 int      _save;
 uint32_t devicetype;
 uint8_t  nodeid;
 uint16_t baudrate;
 uint16_t version;
 uint16_t motortype;
 uint16_t contilimit;
 uint16_t outputilimit;
 uint8_t  polepairnumber;
 uint16_t maxspeedcurrentmode;
 uint16_t thermaltimeconstantwinding;
 uint16_t encoderpulsenr;
 uint16_t positionsensortype;
 int16_t  currentregulatorpgain;
 int16_t  currentregulatorigain;
 int16_t  velocityregulatorpgain;
 int16_t  velocityregulatorigain;
 int16_t  posregulatorpgain;
 int16_t  posregulatorigain;
 int16_t  posregulatordgain;
 int8_t   setoperationmode;
 uint32_t maxprofilevelocity;
 uint32_t profileacc;
 uint32_t profiledec;
 uint32_t quickstopdec;
 int16_t  motionprofiletype;
 uint16_t controlword;
 int32_t  targetvelocity;
 uint16_t statusword;
 int32_t  velsensoractvalue;
 uint16_t misconfiguration;
 uint16_t currentactvalue;
 int32_t  velmodesetvalue;
 


};





struct EposStruct {
  EposParameters parameters;
  BVCanOpenMessageSet set1;
  unsigned int statusWord2;

  bool resetflag;
  int bootcounter;
  int eposIndex;
  int vel;
  int val;			// not change the parameter
  char filename[50];
  int actualvel;

  void init (int index) {
    eposIndex = index;
	
    sprintf(filename,"epos%i.cfg",eposIndex);
    vel = 0;
    resetflag = 0;
    bootcounter = 0;

    BV_DEBUGINFO4 ("-----------------------------");

    readParameters();
  }


  int commitParameters () {
	

parameters.contilimit=7002;
parameters.outputilimit=25000;

    


}


  int check_web_inputs () {



/*
 parameters._read = CONFIG_GETVAR (filename, "_read", 0, "read current paramaters to Epos", 0, 1);
 parameters._set = CONFIG_GETVAR (filename, "_set", 0, "write current paramaters to Epos", 0, 1);
 parameters._save = CONFIG_GETVAR (filename, "_save", 0, "save current paramaters in Epos", 0, 1); 
*/
}


  int readParameters () {


/*

 parameters._read = CONFIG_GETVAR (filename, "_read", 0, "read current paramaters to Epos", 0, 1);
 parameters._set = CONFIG_GETVAR (filename, "_set", 0, "write current paramaters to Epos", 0, 1);
 parameters._save = CONFIG_GETVAR (filename, "_save", 0, "save current paramaters in Epos", 0, 1); 
 parameters.devicetype = CONFIG_GETVAR (filename, "devicetype", 0, "devicetype", 0, 10000);
 parameters.nodeid  = CONFIG_GETVAR (filename, "nodeid", 0, "nodeid", 0, 255);
 parameters.baudrate  = CONFIG_GETVAR (filename, "baudrate", 0, "baudrate", 0, 65535);
 parameters.version  = CONFIG_GETVAR (filename, "version", 0, "version", 0, 65535);
 parameters.motortype  = CONFIG_GETVAR (filename, "motortype", 0, "motortype", 0, 65535);
 parameters.contilimit  = CONFIG_GETVAR (filename, "contilimit", 0, "continuous current limit", 0, 65535);
 parameters.outputilimit  = CONFIG_GETVAR (filename, "outputilimit", 0, "outputilimit", 0, 65535);
 parameters.polepairnumber  = CONFIG_GETVAR (filename, "polepairnumber", 0, "polepairnumber", 0, 255);
 parameters.maxspeedcurrentmode  = CONFIG_GETVAR (filename, "maxspeedcurrentmode", 0, "maxspeedcurrentmode", 0, 65535);
 parameters.thermaltimeconstantwinding  = CONFIG_GETVAR (filename, "thermaltimeconstantwinding", 0, "thermaltimeconstantwinding", 0, 65535);
 parameters.encoderpulsenr  = CONFIG_GETVAR (filename, "encoderpulsenr", 0, "encoderpulsenr", 0, 65535);
 parameters.positionsensortype  = CONFIG_GETVAR (filename, "positionsensortype", 0, "positionsensortype", 0, 65535);
 parameters.currentregulatorpgain  = CONFIG_GETVAR (filename, "currentregulatorpgain", 0, "currentregulatorpgain", -32768, 32767);
 parameters.currentregulatorigain  = CONFIG_GETVAR (filename, "currentregulatorigain", 0, "currentregulatorigain", -32768, 32767);
 parameters.velocityregulatorpgain  = CONFIG_GETVAR (filename, "velocityregulatorpgain", 0, "velocityregulatorpgain", -32768, 32767);
 parameters.velocityregulatorigain  = CONFIG_GETVAR (filename, "velocityregulatorigain", 0, "velocityregulatorigain", -32768, 32767);
 parameters.posregulatorpgain  = CONFIG_GETVAR (filename, "posregulatorpgain", 0, "posregulatorpgain", -32768, 32767);
 parameters.posregulatorigain  = CONFIG_GETVAR (filename, "posregulatorigain", 0, "posregulatorigain", -32768, 32767);
 parameters.posregulatordgain  = CONFIG_GETVAR (filename, "posregulatordgain", 0, "posregulatordgain", -32768, 32767);
 parameters.setoperationmode  = CONFIG_GETVAR (filename, "setoperationmode", 0, "setoperationmode", -128, 127);
 parameters.maxprofilevelocity  = CONFIG_GETVAR (filename, "maxprofilevelocity", 0, "maxprofilevelocity", 0, 10000);
 parameters.profileacc  = CONFIG_GETVAR (filename, "profileacc", 0, "profileacc", 0, 10000);
 parameters.profiledec  = CONFIG_GETVAR (filename, "profiledec", 0, "profiledec", 0, 10000);
 parameters.quickstopdec  = CONFIG_GETVAR (filename, "quickstopdec", 0, "quickstopdec", 0, 10000);
 parameters.motionprofiletype  = CONFIG_GETVAR (filename, "motionprofiletype", 0, "motionprofiletype", -32768, 32767);
 parameters.controlword  = CONFIG_GETVAR (filename, "controlword", 0, "controlword", 0, 65535);
 parameters.targetvelocity  = CONFIG_GETVAR (filename, "targetvelocity", 0, "targetvelocity", -500000, 500000);
 parameters.statusword  = CONFIG_GETVAR (filename, "statusword", 0, "statusword", 0, 65535);
 parameters.velsensoractvalue  = CONFIG_GETVAR (filename, "velsensoractvalue", 0, "velsensoractvalue", -500000, 500000);
 parameters.misconfiguration  = CONFIG_GETVAR (filename, "misconfiguration", 0, "misconfiguration", 0, 65535);
 parameters.currentactvalue  = CONFIG_GETVAR (filename, "currentactvalue", 0, "currentactvalue", 0, 65535);
 parameters.velmodesetvalue  = CONFIG_GETVAR (filename, "velmodesetvalue", 0, "velmodesetvalue", -500000, 500000);
 
 */
 
}

  int update (int setNextVel) {

//    cout << "Epos: " << eposIndex << " - ";

    int all_ok = false;
    vel = setNextVel;
    int timeout =1000;
// CONFIG_GETVAR ("motionServer.cfg", "timeout", 500, "timeout_Canbus",0,   10000);
    unsigned int statusWord;


    BVCanOpenObject * canOpenObject;
    BVMessageTPCANMsg msgReceive;
    
    canOpenObject= set1.objs[ID_BVCanOpenMessageStatusWord];

    canOpenObject->prepareToSend (eposIndex, BVCanOpenObject::read, &val);

    //-----------------------------------------------------------------------------
    // send "Status Word" 
    //-----------------------------------------------------------------------------
    BV_DEBUGINFO4 ("SENDING getStatus");

    can.write (&(canOpenObject->getMsg().objectToString()));

    //-----------------------------------------------------------------------------
    // Confirmation from EPOS 
    //-----------------------------------------------------------------------------
    if (can.wait (BVfd::readable, 1))	// dont ask wha 1 ... I dont know
    {

//      dout << "	Sending ........................" << endl;
      read_can_answer (&msgReceive);
      //-----------------------------------------------------------------------------
      //initialize the canOpenObj2 with the incoming message
      //-----------------------------------------------------------------------------
      canOpenObject->setMsg (&msgReceive);
      //-----------------------------------------------------------------------------
      // check the incoming message for the EPOS status
      //-----------------------------------------------------------------------------
      if (canOpenObject->getCanOpenObjectID () == ID_BVCanOpenMessageStatusWord) {
//	printf ("StatusWord :%x \n", statusWord);
 	statusWord= canOpenObject->getValU32 ();


	switch (statusWord & 0x417f) {
	case 0x0000:
	  dout << " BOOTUP" << endl;
	  bootcounter++;
	  break;
	case 0x0100:
	  dout << " NOT READY TO SWITCH ON" << endl;
	  break;
	case 0x0140:
	  dout << " SWITCH ON DISABLED" << endl;;
	  break;
	case 0x0121:
	  dout << " READY TO SWITCH ON" << endl;
	  break;
	case 0x0123:
	  dout << " SWITCHED ON" << endl;
	  break;
	case 0x4123:
	  dout << " REFRESH" << endl;
	  break;
	case 0x4133:
	  dout << " MEASURE UNIT" << endl;
	  break;
	case 0x0137:
//	  dout << " OPERATION ENABLED" << endl; Suppress Operation enabled
	  break;
	case 0x0117:
	  dout << " FAULT REACTION ACTIVE (DISABLED)" << endl;
	  break;
	case 0x011F:
	  dout << " FAULT REACTION ACTIVE (ENABLED)" << endl;
	  break;
	case 0x0108:
	  dout << " FAULT" << endl;
	  break;
	case FAULT_OVER_CURRENT:
	  dout << " FAULT OVER CURRENT" << endl;
	  break;
	default:
	  cout << " UNKNOWN statusWORD=" << hex << statusWord << dec << endl;

	  break;
	}			//switch
      }				//if
    }				//if

    else {
      cout << "timeout epos " << eposIndex;
    }



    if (((statusWord & 0x417f) == FAULT)|| ((statusWord &0x417)== FAULT_REACTION_ACTIVE_DISABLED )  || (statusWord &0x417)== FAULT_REACTION_ACTIVE_ENABLED           ){
    //if (((statusWord2 & 0x417f) == FAULT) )
//    if (statusWord2 &0x417f == 0x0) {

	cerr << "trying to reset" << endl;

	int cw = 0x80;
        canOpenObject= set1.objs[ID_BVCanOpenMessageControlWord];
        canOpenObject->prepareToSend (eposIndex, BVCanOpenObject::write, &cw);
	BV_DEBUGINFO4 ("SENDING RESET");
    	can.write (&(canOpenObject->getMsg().objectToString()));


    }

/*

    //alle fault bedingungen abfangen
    if (((statusWord2 & 0x8) == 0x8))	//ist das 4. bit an?     
    {
      cout << "Reacting to fault bit" << endl;
      int val;
      BVCanOpenObject *outCanOpenObject ;
      outCanOpenObject      = set1.objs[ID_BVCanOpenMessageCobIDEmcy];
      
      outCanOpenObject->prepareToSend(eposIndex ,   BVCanOpenObject::read , &val);
      BV_DEBUGINFO4("Sending Ask for Value=[" << val << "]");
      can.write( &(outCanOpenObject->getMsg().objectToString()));
      
      if(can.wait( BVfd::readable , 500)){
	read_can_answer(&msgReceive);



	printf("Hex 5 %x byte 6 %x \n ",msgReceive.getDATA(4),msgReceive.getDATA(5));
	printf("Byte 5 %i byte 6 %i \n ",msgReceive.getDATA(4),msgReceive.getDATA(5));


//	parameters.outputilimit=msgReceive.getDATA(5)*256+msgReceive.getDATA(4);


        }
        else
	  BV_WARNING("timeout");






      int cw = 0x80;
        canOpenObject= set1.objs[ID_BVCanOpenMessageControlWord];
        canOpenObject->prepareToSend (eposIndex, BVCanOpenObject::write, &cw);
	BV_DEBUGINFO4 ("SENDING RESET");
    	can.write (&(canOpenObject->getMsg().objectToString()));
      can.write( &(outCanOpenObject->getMsg().objectToString()));
      if (can.wait (BVfd::readable, timeout)) {

	read_can_answer (&msgReceive);

      }

      else {

	dout << "timeout epos " << eposIndex;
      }
    }
*/	

    //-----------------------------------------------------------------------------    
    // "Sending Control Word SWITCH_ON_DISABLED--->READY_TO_SWITCH_ON" 
    //-----------------------------------------------------------------------------
    if ((statusWord & 0x417f) == SWITCH_ON_DISABLED) {
      int cw = 0x06;  // SEND READY TO SWITCH ON
      canOpenObject= set1.objs[ID_BVCanOpenMessageControlWord];
      canOpenObject->prepareToSend (eposIndex, BVCanOpenObject::write, &cw);
      dout << "SENDING CW0 (Shutdown) " << endl;
      can.write (&(canOpenObject->getMsg().objectToString()));

      if (can.wait (BVfd::readable, timeout))
	read_can_answer (&msgReceive);
      else
	dout << "timeout epos " << eposIndex;


    }
  

  //-----------------------------------------------------------------------------    
    // "Sending Control Word READY_TO_SWITCH_ON--->SWITCHED_ON 
    //-----------------------------------------------------------------------------
    if ((statusWord & 0x417f) == READY_TO_SWITCH_ON) {
      int cw = 0x07; // SEND SWITCH ON 
      canOpenObject= set1.objs[ID_BVCanOpenMessageControlWord];
      canOpenObject->prepareToSend (eposIndex, BVCanOpenObject::write, &cw);
      dout << "SENDING CW1 (SwitchedOn) " << endl;
      can.write (&(canOpenObject->getMsg().objectToString()));

      if (can.wait (BVfd::readable, timeout)) 
	read_can_answer (&msgReceive);
      else
	dout << "timeout epos " << eposIndex;



    }
    //-----------------------------------------------------------------------------    
    // "Sending Control Word VELCOCITY MODE
    //-----------------------------------------------------------------------------

    if ((statusWord & 0x417f) == SWITCHED_ON) {
      int cw = 0x0f;  // SEND VEL MODE
      canOpenObject= set1.objs[ID_BVCanOpenMessageControlWord];
      canOpenObject->prepareToSend (eposIndex, BVCanOpenObject::write, &cw);
//      dout << "SENDING CW2 (enabledOP) " << endl;
      can.write (&(canOpenObject->getMsg().objectToString()));

      if (can.wait (BVfd::readable, timeout)) 
	read_can_answer (&msgReceive);
      else
	dout << "timeout epos " << eposIndex;
       


    }
    //-----------------------------------------------------------------------------    
    // "Sending Target Velocity
    //-----------------------------------------------------------------------------
    if ((statusWord & 0x417f) == OPERATION_ENABLED) {

      canOpenObject = set1.objs[ID_BVCanOpenMessageTargetVelocity];
      canOpenObject->prepareToSend (eposIndex, BVCanOpenObject::write, &vel);
//      dout << "Sending Target Velocity=[" << vel << "]";
//      cout << "Sending Target Velocity" << " to Epos " << eposIndex <<
    //    " [" << vel << "]" << endl;;
      can.write (&(canOpenObject->getMsg().objectToString()));

      if (can.wait (BVfd::readable, timeout)) {
	read_can_answer (&msgReceive);

//      all_ok=true;


	int cw = 0x0f;
      	canOpenObject= set1.objs[ID_BVCanOpenMessageControlWord];
	canOpenObject->prepareToSend (eposIndex, BVCanOpenObject::write, &cw);
//	dout << "SENDING CW2 (enabledOP) " << endl;
         can.write (&(canOpenObject->getMsg().objectToString()));

	if (can.wait (BVfd::readable, timeout)) {
	  all_ok = true;	//in diesem fall funtkioniert das epos
	  read_can_answer (&msgReceive);

	}

	else
	  BV_WARNING ("timeout");


//      exitFlag=true;  
      }

      else
	dout << "timeout epos " << eposIndex;
      //             exitFlag=true;


    }
/*    if(true)
	{
		for (int i =0;i<35;i++){

      canOpenObject      = set1.objs[i];
      canOpenObject->prepareToSend(eposIndex ,   BVCanOpenObject::read , &val);
      can.write( &(canOpenObject->getMsg().objectToString()));
     if(can.wait( BVfd::readable , 500)){
	read_can_answer(&msgReceive);
	int data4        = (int)msgReceive.getDATA(4);
    	int data5        = (int)msgReceive.getDATA(5);
    	int data6        = (int)msgReceive.getDATA(6);
    	int data7        = (int)msgReceive.getDATA(7);

	string mname;
	set1.getNameOfObject(i , &mname);
	printf("\n%s is %i",mname.c_str(),actualvel);
	}
        else
	  BV_WARNING("timeout");


	}
	}
	//printf (" \n");
*/
    if(get_odometry)
	{
	  canOpenObject      = set1.objs[ID_BVCanOpenMessageVelSensorActValue];
      canOpenObject->prepareToSend(eposIndex ,   BVCanOpenObject::read , &val);
      //cout<<"\nSending Target Velocity=[" << vel << "]"<<endl;;

  //   StopWatch sw;

     can.write( &(canOpenObject->getMsg().objectToString()));
//     cout << "Can write took "<<sw.get_usecs()<<endl;      
//     usleep(1800);
   //  sw.reset();
     if(can.wait( BVfd::readable , 500)){
	read_can_answer(&msgReceive);
 //    cout << "Can read took "<<sw.get_usecs()<<endl;      

    int data4        = (int)msgReceive.getDATA(4);
    int data5        = (int)msgReceive.getDATA(5);
    int data6        = (int)msgReceive.getDATA(6);
    int data7        = (int)msgReceive.getDATA(7);

    int v =  (data7 << 24) + (data6 << 16) +( data5 << 8) + (data4);
      v= v<=0x7FFFFFFF ? v : -(0xFFFFFFFF-v) ;
	actualvel=(v)*3.0f/100; // correction for quadcounts in encoder
//	printf("\nActual vel is %i",actualvel);
        }

        else
	  BV_WARNING("timeout");



	}
	//printf (" \n");
    return all_ok;
  }





int saveParameters(){
      BVMessageTPCANMsg msgReceive;
      BVCanOpenObject *outCanOpenObject      = set1.objs[ID_BVCanOpenMessageSave];
	unsigned long v=1702257011;
      //valsend=('s'<<24 | 'a'<< 16 | 'v'<< 8 <<'e');
       int val=v;
      cout << "savething is ************************************* "<<val<<endl;
	outCanOpenObject->prepareToSend(eposIndex ,   BVCanOpenObject::write , &val);
      BV_DEBUGINFO4("Sending Save-Message=[" << val << "] ***************************************************************");
      //cout<<"Sending Target Velocity=[" << vel << "]"<<endl;;
      can.write( &(outCanOpenObject->getMsg().objectToString()));
//      BV_DEBUGINFO4C(" data="<< (int)msgSend5.getDATA(0) << "]");
      
      if(can.wait( BVfd::readable , 500)){
	read_can_answer(&msgReceive);
    
	cout << "saving Ready     ***********************************************"<<endl;
        }

        else
	  BV_WARNING("timeout");
}





  int update_parameters () {
    


 BVCanOpenObject * canOpenObj1;
 BVCanOpenObject * canOpenObj2;
 BVCanOpenObject * cw0;
 BVCanOpenObject * cw1;
 BVCanOpenObject * cw2;
 BVCanOpenObject * initCWReset;
 BVCanOpenObject * velMsg;

    canOpenObj1 = set1.objs[ID_BVCanOpenMessageStatusWord];
    canOpenObj2 = set1.objs[ID_BVCanOpenMessageStatusWord];
    cw0 = set1.objs[ID_BVCanOpenMessageControlWord];
    cw1 = set1.objs[ID_BVCanOpenMessageControlWord];
    cw2 = set1.objs[ID_BVCanOpenMessageControlWord];
    initCWReset = set1.objs[ID_BVCanOpenMessageControlWord];
    velMsg = set1.objs[ID_BVCanOpenMessageTargetVelocity];



  readParameters();


  if (parameters._read>0 || parameters._set>0||parameters._save){ 


  val=0;
  canOpenObj1->prepareToSend( eposIndex ,   BVCanOpenObject::read , &val);
  BVMessageTPCANMsg msgSend1 = canOpenObj1->getMsg();
  
  bool exitFlag=false;

//	usleep(30000);
	usleep(1000000);
    //-----------------------------------------------------------------------------
    // send "Status Word" 
    //-----------------------------------------------------------------------------
    BV_DEBUGINFO4("SENDING getStatus");
    string strSend1 = msgSend1.objectToString();
    can.write( &strSend1 );
    //-----------------------------------------------------------------------------
    // Confirmation from EPOS 
    //-----------------------------------------------------------------------------
	BVMessageTPCANMsg msgReceive;
    if(can.wait( BVfd::readable , 5)){
    
      read_can_answer(&msgReceive);
      //-----------------------------------------------------------------------------
      //initialize the canOpenObj2 with the incoming message
      //-----------------------------------------------------------------------------
      canOpenObj2->setMsg(&msgReceive); 
      //-----------------------------------------------------------------------------
      // check the incoming message for the EPOS status
      //-----------------------------------------------------------------------------
      if(canOpenObj2->getCanOpenObjectID() == ID_BVCanOpenMessageStatusWord ){
        unsigned int statusWord = canOpenObj2->getValU32();
        statusWord2 = statusWord;
        
        switch( statusWord & 0x417f  ){
        case 0x0000:
          BV_DEBUGINFO4C( " BOOTUP" << endl );
          break;
        case 0x0100:
          BV_DEBUGINFO4C( " NOT READY TO SWITCH ON" << endl );
          break;
        case 0x0140:
          BV_DEBUGINFO4C( " SWITCH ON DISABLED"  << endl);
          break;
        case 0x0121:
          BV_DEBUGINFO4C( " READY TO SWITCH ON" << endl );
          break;
        case 0x0123:
          BV_DEBUGINFO4C( " SWITCHED ON" << endl );
          break;
        case 0x4123:
          BV_DEBUGINFO4C(" REFRESH" << endl  );
          break;
        case 0x4133:
          BV_DEBUGINFO4C( " MEASURE UNIT"  << endl );
          break;
        case 0x0137:
          BV_DEBUGINFO4C(" OPERATION ENABLED" << endl);
          break;
        case 0x0117:
          BV_DEBUGINFO4C(" FAULT REACTION ACTIVE (DISABLED)" << endl);
          break;
        case 0x011F:
          BV_DEBUGINFO4C(" FAULT REACTION ACTIVE (ENABLED)" << endl);
          break;
        case 0x0108:
          BV_DEBUGINFO4C(" FAULT" << endl);
          break;
        default:
          BV_WARNINGC(" UNKNOWN statusWORD=" << hex << statusWord  << dec << endl);
          break;
        }  //switch
      } //if
    } //if

    else{
      BV_WARNING("timeout");
    }

    //-----------------------------------------------------------------------------    
    // "Sending reset" 
    //-----------------------------------------------------------------------------
    if ((statusWord2 & 0x417f) == FAULT){
      	
//	(Im falle von Fault erstmal Fehler abfragen)      


      int cw = 0x80;
      initCWReset->prepareToSend( eposIndex ,   BVCanOpenObject::write , &cw );
      BVMessageTPCANMsg msgSend2 = initCWReset->getMsg();
      string strSend2 = msgSend2.objectToString();
      BV_DEBUGINFO4("SENDING RESET");
      can.write( &strSend2 );
      
      if(can.wait( BVfd::readable , 500)){
      	
	read_can_answer(&msgReceive);
    
      }

      else
	BV_WARNING("timeout");
    }  

    //-----------------------------------------------------------------------------    
    // "Sending Control Word SWITCH_ON_DISABLED--->READY_TO_SWITCH_ON" 
    //-----------------------------------------------------------------------------
    if ((statusWord2 & 0x417f) == SWITCH_ON_DISABLED){
      int cw = 0x06;
      cw0->prepareToSend( eposIndex ,   BVCanOpenObject::write , &cw );
      BVMessageTPCANMsg msgSend2 = cw0->getMsg();
      string strSend2 = msgSend2.objectToString();
      BV_DEBUGINFO4("SENDING CW0 (Shutdown) ");
      can.write( &strSend2 );
      
      string strRecieve2;
      if(can.wait( BVfd::readable , 500)){
    
	read_can_answer(&msgReceive);
      }

      else
	BV_WARNING("timeout");
    }  
    //-----------------------------------------------------------------------------    
    // "Sending Control Word READY_TO_SWITCH_ON--->SWITCHED_ON 
    //-----------------------------------------------------------------------------
    if ((statusWord2 & 0x417f) == READY_TO_SWITCH_ON){
      int cw = 0x07;
      cw1->prepareToSend(eposIndex ,   BVCanOpenObject::write , &cw );
      BVMessageTPCANMsg msgSend3 = cw1->getMsg();
      string strSend3 = msgSend3.objectToString();
      BV_DEBUGINFO4("SENDING CW1 (SwitchedOn) ");
      can.write( &strSend3 );

      string strRecieve3;
      if(can.wait( BVfd::readable , 500)){
    
	read_can_answer(&msgReceive);
      }

      else
	BV_WARNING("timeout");
    }
    //-----------------------------------------------------------------------------    
    // "Sending Control Word SWITCHED_ON--->OPERATION_ENABLED
    //-----------------------------------------------------------------------------

    if ((statusWord2 & 0x417f) == SWITCHED_ON){
      int cw = 0x0f;
      cw2->prepareToSend( eposIndex ,   BVCanOpenObject::write , &cw );
      BVMessageTPCANMsg msgSend4 = cw2->getMsg();
      string strSend4 = msgSend4.objectToString();
      BV_DEBUGINFO4("SENDING CW2 (enabledOP) ");
      can.write( &strSend4 );
      
      string strRecieve4;
      if(can.wait( BVfd::readable , 500)){
	read_can_answer(&msgReceive);
    
      }

      else
	BV_WARNING("timeout");
    }



    //-----------------------------------------------------------------------------    
    // "Sending Control Word VELCOCITY MODE
    //-----------------------------------------------------------------------------

    if ((statusWord2 & 0x417f) == SWITCHED_ON){
      int cw = 0x0f;
      cw2->prepareToSend( eposIndex ,   BVCanOpenObject::write , &cw );
      BVMessageTPCANMsg msgSend4 = cw2->getMsg();
      string strSend4 = msgSend4.objectToString();
      BV_DEBUGINFO4("SENDING CW2 (enabledOP) ");
      can.write( &strSend4 );
      
      string strRecieve4;
      if(can.wait( BVfd::readable , 500)){
	read_can_answer(&msgReceive);
    
      }

      else
	BV_WARNING("timeout");
    }





    if ((statusWord2 & 0x417f) == OPERATION_ENABLED){

	
      BVCanOpenObject *outCanOpenObject;
      int val=0;


      outCanOpenObject      = set1.objs[ID_BVCanOpenMessageOutputCurrentLimit];
      if (parameters._set){
      val=parameters.outputilimit;	
      outCanOpenObject->prepareToSend(eposIndex ,   BVCanOpenObject::write , &val);
      BV_DEBUGINFO4("Sending new Value=[" << val << "]");
      can.write( &(outCanOpenObject->getMsg().objectToString()) );
      if(can.wait( BVfd::readable , 500))
        	read_can_answer(&msgReceive);
      else
                BV_WARNING("timeout");
     
      }
 
      outCanOpenObject->prepareToSend(eposIndex ,   BVCanOpenObject::read , &val);
      BV_DEBUGINFO4("Sending Ask for Value=[" << val << "]");
      can.write( &(outCanOpenObject->getMsg().objectToString()));
      
      if(can.wait( BVfd::readable , 500)){
	read_can_answer(&msgReceive);
	parameters.outputilimit=msgReceive.getDATA(5)*256+msgReceive.getDATA(4);
	printf ("Output Stromlimit ist %i \n",parameters.outputilimit);


        }
        else
	  BV_WARNING("timeout");


      outCanOpenObject      = set1.objs[ID_BVCanOpenMessageContinousCurrentLimit];
      if (parameters._set){
       val=parameters.contilimit;
      outCanOpenObject->prepareToSend(eposIndex ,   BVCanOpenObject::write , &val);
      BV_DEBUGINFO4("Sending new Value=[" << val << "]");
      can.write( &(outCanOpenObject->getMsg().objectToString()));
      
      if(can.wait( BVfd::readable , 500)){
	read_can_answer(&msgReceive);
    

        }

        else
	  BV_WARNING("timeout");
      }
//	exitFlag=true;  
     
      outCanOpenObject->prepareToSend(eposIndex ,   BVCanOpenObject::read , &val);
      BV_DEBUGINFO4("Sending Ask for Value=[" << val << "]");
      //cout<<"Sending Target Velocity=[" << vel << "]"<<endl;;
      can.write( &(outCanOpenObject->getMsg().objectToString()));
      
      if(can.wait( BVfd::readable , 500)){
	read_can_answer(&msgReceive);
   
	//printf("Hex 5 %x byte 6 %x \n ",msgReceive.getDATA(4),msgReceive.getDATA(5));
	//printf("Byte 5 %i byte 6 %i \n ",msgReceive.getDATA(4),msgReceive.getDATA(5));
	parameters.contilimit=msgReceive.getDATA(5)*256+msgReceive.getDATA(4);
	printf ("Stromlimit ist %i \n",parameters.contilimit);

        }

        else
	  BV_WARNING("timeout");
	
//
//
// Save the stuff; 
//
//  
     if (parameters._save ) saveParameters();

}


commitParameters();

}// (if read)

/*
if (parameters._read>0){ parameters._read=0;CONFIG_SETVAR (filename, "_read", 0);}
if (parameters._save>0){ parameters._save=0;CONFIG_SETVAR (filename, "_save", 0);}
if (parameters._set>0){ parameters._set=0;CONFIG_SETVAR (filename, "_set", 0);}
*/





}




};



