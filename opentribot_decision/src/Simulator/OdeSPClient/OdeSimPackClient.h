#ifndef _ODESIMPACK_CLIENT_H_
#define _ODESIMPACK_CLIENT_H_

#include <string>
#include "../../Structures/TribotsException.h"
#include "../../Fundamental/ConfigReader.h"
#include "../../Structures/DriveVector.h"
#include "../../Fundamental/Time.h"
#include "SimSocket.h"


namespace Tribots {

  struct OdeSimPackClientData {
    float RobotPos[3];
    float RobotAbsVel[3];
    float RobotRelVel[3];
    float BallAbsPos[3];
    float BallAbsVel[3];
    std::vector< float > Obstacles;
    Time  timestamp;
  };

  
  class OdeSimPackClient {
    
  public:
    static OdeSimPackClient* getTheClient(const Tribots::ConfigReader* cfg=0) throw(Tribots::TribotsException);

    ~OdeSimPackClient() throw();

    void receiveData() throw(Tribots::BadHardwareException, Tribots::HardwareException);
    const OdeSimPackClientData& getLastSimData() { return simData;};

    void sendDriveVector(const Tribots::DriveVector& dv) throw(Tribots::BadHardwareException);

    bool sendBallRelocationRequest(double x, double y, double z, double xp , double yp , double zp);
    bool sendRobotRelocationRequest(double x, double y, double phi);
    
  protected:
    OdeSimPackClient(const Tribots::ConfigReader* cfg = 0) throw(Tribots::TribotsException);
    
    static OdeSimPackClient* theOnlyClient;

    std::string    servAddr;
    unsigned short servPort;

    std::string    serverType;
    std::string    ObjID;

    SimUtils::SimSocket* clntSocket;

    OdeSimPackClientData simData;
    

    bool goalie;
  };
}


#endif
