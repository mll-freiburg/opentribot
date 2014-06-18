#ifndef _SIM_SOCKET_H_
#define _SIM_SOCKET_H_

#include "Socket.h"
#include <vector>

#define MAX_DATA 1024

namespace SimUtils {
  
  enum DataTypeTag {DTypeString=0, DTypeFloat=1, DTypeChar=2, DTypeListOfFloat=3};

  class ProtocolPackage 
  {
    friend class SimSocket;
    
  public:
    ProtocolPackage(int _maxBufferLen=1024);
    ~ProtocolPackage();
    
    int getLength() const;
    const char* getBuffer() const;
    void clear();

    void put(std::string s);
    void put(float v);
    void put(char v);
    void put(const std::vector< float >& v);

    void startEval();
    bool nextEvalString(std::string &vs);
    bool nextEvalFloat(float &v);
    bool nextEvalChar(char &v);
    bool nextEvalListOfFloat(std::vector< float >& v);

  protected:
    static const int headerLength = sizeof(int);
    int   maxBufferLen;
    int   dataLength;
    char* buffer;
    int   evalCtr;
  };


  class SimSocket : public TCPSocket
  {
  public:
    SimSocket(const std::string &foreignAddress, 
	      unsigned short foreignPort) throw(SocketException) ;
    
    bool send    (const ProtocolPackage  &pckg) throw(SocketException);
    bool receive (      ProtocolPackage  &pckg) throw(SocketException);
    
    // intitialized by server
    bool initHandshake(const std::string& serverName, std::string& clientName) throw(SocketException);
    bool waitForHandshake(const std::string& clientName, std::string& serverName) throw(SocketException);
    
  private:
    static const float SOCKVER;// = 0.1;

    friend class SimServer;
    SimSocket(int newSocketDesc);
  };
  
}
#endif
