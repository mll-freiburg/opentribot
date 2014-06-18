#include "OnlineRobotMonitorClient.h"

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <iostream>

#define NP1             "/tmp/OnlineRobotMonitorPipe1"
#define NP2             "/tmp/OnlineRobotMonitorPipe2"

Tribots::OnlineRobotMonitorClient::OnlineRobotMonitorClient()
{
  wrfd = -1;
  rdfd = -1;
  hasconnection = false;
  datamutex        = (pthread_mutex_t) PTHREAD_MUTEX_INITIALIZER;
  
  int ret_val;
  /* Create the first named - pipe */
  ret_val = mkfifo(NP1, 0666);
  if ((ret_val == -1) && (errno != EEXIST)) {
    perror("Error creating the named pipe");
    exit (1);
  }
  
  ret_val = mkfifo(NP2, 0666);
  if ((ret_val == -1) && (errno != EEXIST)) {
    perror("Error creating the named pipe");
    exit (1);
  }
  
}

Tribots::OnlineRobotMonitorClient::~OnlineRobotMonitorClient()
{
  if (wrfd > 0) close(wrfd);
  if (rdfd > 0) close(rdfd);
}

bool Tribots::OnlineRobotMonitorClient::startcom()
{
    if (hasconnection) return true;
    /* Open the first named pipe for writing */
    std::cerr << "opening np2 for writing\n";
    wrfd = open(NP2, O_WRONLY);
    write(wrfd, "OPEN", 4);	
    
    std::cerr << "opening np1 for reading\n";
    rdfd = open(NP1, O_RDONLY);	
    
    hasconnection = true;
    int ret = pthread_create( &theThread, NULL, comThread, (void*) this);
    
    return ( ret == 0);
}

bool Tribots::OnlineRobotMonitorClient::stopcom()
{
    if (!hasconnection) return true;
    
    write(wrfd,"CLOSE", 5);
    close(wrfd);
    wrfd = -1;
    
    pthread_join( theThread, NULL); 
    
    close(rdfd);
    rdfd = -1;
    
    hasconnection = false;
    return true;
}	

bool Tribots::OnlineRobotMonitorClient::receive()
{
    if (!hasconnection) return false;
    int numread;
    numread = read(rdfd, readbuf, MAX_PIPEBUF_LEN);
    readbuf[numread]='\0';
    //std::cerr << "Received: " << readbuf << "\n";
    
    if (strncmp(readbuf, "CLOSE", 5)==0) {
	return false;
    }
    
    RobotMonitorData data;
    sscanf(readbuf, "ROBOTDATA %f %f %f %f %f %f %f %f %f %f %f %f", 
	   &data.wheel_vel_ist[0], 
	   &data.wheel_vel_ist[1], 
	   &data.wheel_vel_ist[2],
	   &data.wheel_vel_soll[0], 
	   &data.wheel_vel_soll[1], 
	   &data.wheel_vel_soll[2],
	   &data.robot_vel_ist[0], 
	   &data.robot_vel_ist[1], 
	   &data.robot_vel_ist[2],
	   &data.robot_vel_soll[0], 
	   &data.robot_vel_soll[1], 
	   &data.robot_vel_soll[2]
	   );
    
    pthread_mutex_lock( &datamutex );
    receivedData.push_back(data);
    pthread_mutex_unlock( &datamutex );
    
    return true;
}

void * Tribots::OnlineRobotMonitorClient::comThread(void * OnlineRobotMonitorClient_instance)
{
    Tribots::OnlineRobotMonitorClient* instance = (OnlineRobotMonitorClient*) OnlineRobotMonitorClient_instance;
    
    while( instance->receive() );

}

 void Tribots::OnlineRobotMonitorClient::getNewData(std::vector< RobotMonitorData >& data)
 {
     data.clear();
     pthread_mutex_lock( &datamutex );
     for (unsigned int i=0; i<receivedData.size(); i++)
	 data.push_back(receivedData[i]);
     receivedData.clear();
    pthread_mutex_unlock( &datamutex );
 }
 
 void  Tribots::OnlineRobotMonitorClient::sendPIDSettings( unsigned int kp, unsigned int ki, unsigned int kd)
 {
     sprintf(writebuf, "PID %ud %ud %ud", kp, ki, kd);
     write( wrfd, writebuf, strlen(writebuf));
 }
