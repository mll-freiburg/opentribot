#ifndef _ONLINE_ROBOTMONITOR_CLIENT_H_
#define _ONLINE_ROBOTMONITOR_CLIENT_H_

#define MAX_PIPEBUF_LEN 1024

#include <vector>

namespace Tribots {
    
    struct RobotMonitorData {
	float wheel_vel_ist[3];
	float wheel_vel_soll[3];
	float robot_vel_ist[3];
	float robot_vel_soll[3];
    };
    
  class OnlineRobotMonitorClient {
  public:
    OnlineRobotMonitorClient();
    ~OnlineRobotMonitorClient();
    
    bool startcom();
    bool stopcom();
    
    bool receive();
    
    void getNewData(std::vector< RobotMonitorData >& data);
    
    void sendPIDSettings( unsigned int kp, unsigned int ki, unsigned int kd);
  
protected:
    bool hasconnection;
    int rdfd, wrfd;
    
    char writebuf[MAX_PIPEBUF_LEN];
    char readbuf[MAX_PIPEBUF_LEN];	
    
    pthread_t  theThread;
    pthread_mutex_t      datamutex;
    std::vector< RobotMonitorData > receivedData;
    
    static void * comThread(void * OnlineRobotMonitorClient_instance);	
  };


}

#endif
