#include "OdeSimPackClient.h"
#include <sstream>
#include "../../Structures/Journal.h"

enum ORCmdMode {VELROBOT=0, VELWHEELS=1, VELMOTORS=2, UMOTORS=3, RELOCATEBALL=4, RELOCATEROBOT=5};

Tribots::OdeSimPackClient* Tribots::OdeSimPackClient::theOnlyClient = 0;

Tribots::OdeSimPackClient::OdeSimPackClient(const Tribots::ConfigReader* cfg) throw(Tribots::TribotsException)
{
  serverType = "?";
  ObjID      = "?";

  servAddr   = "127.0.0.1";
  servPort   =  6020;

  goalie     = false;

  float sensorDelay_ms = 0;
  float actionDelay_ms = 0;

  if (cfg!=0) {
    // read the configs
    cfg->get("OdeSimPack::server", servAddr);
    int port;
    if ( cfg->get("OdeSimPack::port", port) ) servPort = (unsigned int) port;
    
    cfg->get("OdeSimPack::goalie", goalie);

    cfg->get("OdeSimPack::sensorDelay_ms", sensorDelay_ms);
    cfg->get("OdeSimPack::actionDelay_ms", actionDelay_ms);
  }
  
  clntSocket = 0;
  try {
    clntSocket = new SimUtils::SimSocket(servAddr, servPort);
    clntSocket->waitForHandshake("RobotControlClient", serverType);
    
    SimUtils::ProtocolPackage ps;
    ps.put("OBJTYPE");
    ps.put("OmniRobot");

    std::stringstream params;
    if (goalie) params << "goalie = true\n";
    params << "sensorDelay_ms = " << sensorDelay_ms << "\n";
    params << "actionDelay_ms = " << actionDelay_ms << "\n";
    ps.put(params.str());
    clntSocket->send(ps);
    
    SimUtils::ProtocolPackage pr;
    std::string res;
    pr.clear();
    clntSocket->receive(pr);
    pr.startEval();
    pr.nextEvalString(res);
    
    if (res != "OK")
      {
	throw Tribots::TribotsException("Simulator refused to create obj.");
      }
    else {
      pr.nextEvalString(res);
      ObjID = res;
      std::stringstream msg;
      msg << "Connected to Simulator and received ID: " << res ;
      JMESSAGE(msg.str().c_str());
    }
    
  }
  catch (SimUtils::SocketException& e) {
    std::stringstream msg;
    msg << "Can't connect client: " << e.what();
    throw Tribots::BadHardwareException(msg.str().c_str());
  }
  
}

Tribots::OdeSimPackClient::~OdeSimPackClient() throw()
{
  if ( clntSocket!= 0 ) delete clntSocket;
}

Tribots::OdeSimPackClient* Tribots::OdeSimPackClient::getTheClient(const Tribots::ConfigReader* cfg) throw(Tribots::TribotsException)
{
  if (theOnlyClient == 0) {
    theOnlyClient = new OdeSimPackClient(cfg);
  }
  return theOnlyClient;
}


void Tribots::OdeSimPackClient::receiveData() throw(Tribots::BadHardwareException, Tribots::HardwareException)
{
  SimUtils::ProtocolPackage pr;
  try {
    clntSocket->receive(pr);
  }
  catch(SimUtils::SocketException& e) {
    
    std::stringstream msg;
    msg << "OdeSPClient was not able to get data: " << e.what();
    throw Tribots::BadHardwareException(msg.str().c_str());
  }  
  
  simData.timestamp.update();
  
  pr.startEval();
  float v[13];
  std::vector< float > Obstacles;

  bool  res   = true;
  for (int h=0; h<13; h++) res&=pr.nextEvalFloat(v[h]);
  res &= pr.nextEvalListOfFloat(simData.Obstacles);
  
  if (!res) {
    std::stringstream msg;
    msg << "OdeSPClient error in data format.";
    throw Tribots::HardwareException(msg.str().c_str());
  }
  
  for (int i=0; i<3; i++) simData.RobotPos[i]    = v[i+0];
  for (int i=0; i<3; i++) simData.RobotAbsVel[i] = v[i+3];
  for (int i=0; i<3; i++) simData.RobotRelVel[i] = v[i+6];
  for (int i=0; i<2; i++) simData.BallAbsPos[i]  = v[i+9];
  for (int i=0; i<2; i++) simData.BallAbsVel[i]  = v[i+11];
  
}


void Tribots::OdeSimPackClient::sendDriveVector(const Tribots::DriveVector& dv) throw(Tribots::BadHardwareException)
{
  SimUtils::ProtocolPackage ps;
  switch (dv.mode) {
  case Tribots::ROBOTVELOCITY :
    ps.put((char) ((int) VELROBOT));
    ps.put((float) dv.vtrans.x);
    ps.put((float) dv.vtrans.y);
    ps.put((float) dv.vrot);
    std::cerr << "Put dv ... : " <<  dv.vtrans.x << "  " << dv.vtrans.y << "\n\r";
    break;
  case Tribots::WHEELVELOCITY :
    ps.put((char) ((int) VELWHEELS));
    ps.put((float) dv.vaux[0]);
    ps.put((float) dv.vaux[1]);
    ps.put((float) dv.vaux[2]);
    break;
  case Tribots::MOTORVOLTAGE  :
    ps.put((char) ((int) UMOTORS));
    ps.put((float) dv.vaux[0]);
    ps.put((float) dv.vaux[1]);
    ps.put((float) dv.vaux[2]);
    break;
  default:
    JWARNING("UNKNOWN MOTOR VEL CMD TYPE IN DV");
    break;
  };

  if (dv.kick) ps.put((float) (dv.klength));
  else ps.put((float) 0.0);

  // TODO Kicker cmd
  
  try {
    clntSocket->send(ps);
  }
  catch (SimUtils::SocketException& e) {
    std::stringstream msg;
    msg << "OdeSPClient was not able to send data: " << e.what();
    throw Tribots::BadHardwareException(msg.str().c_str());
  }    
}


bool Tribots::OdeSimPackClient::sendBallRelocationRequest(double x, double y, double z, double xp , double yp , double zp)
{
  bool res = true;

  SimUtils::ProtocolPackage ps;
  
  ps.put((char) ((int) RELOCATEBALL));
  ps.put((float) x);
  ps.put((float) y);
  ps.put((float) z);
  ps.put((float) xp);
  ps.put((float) yp);
  ps.put((float) zp);
  
  try {
    clntSocket->send(ps);
  }
  catch (SimUtils::SocketException& e) {
    std::stringstream msg;
    msg << "OdeSPClient was not able to send ball relocation request: " << e.what();
    JERROR(msg.str().c_str());
    res = false;
  }    
  return res;
}

bool Tribots::OdeSimPackClient::sendRobotRelocationRequest(double x, double y, double phi)
{
  bool res = true;
  SimUtils::ProtocolPackage ps;
  
  ps.put((char) ((int) RELOCATEROBOT));
  ps.put((float) x);
  ps.put((float) y);
  ps.put((float) phi);
  
  try {
    clntSocket->send(ps);
  }
  catch (SimUtils::SocketException& e) {
    std::stringstream msg;
    msg << "OdeSPClient was not able to send robot relocation request: " << e.what();
    JERROR(msg.str().c_str());
    res = false;
  }    
  return res;
}
