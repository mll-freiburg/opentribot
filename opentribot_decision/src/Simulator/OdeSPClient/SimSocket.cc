#include "SimSocket.h"
#include <iostream>
#include <cmath>

#define MIN(x,y) (x<y?x:y)

const float SimUtils::SimSocket::SOCKVER = 0.1;

SimUtils::ProtocolPackage::ProtocolPackage(int _maxBufferLen)
  : maxBufferLen(_maxBufferLen)
{
  buffer     = new char[maxBufferLen];
  dataLength = 0;
  evalCtr    = 0;
}

SimUtils::ProtocolPackage::~ProtocolPackage()
{
  delete [] buffer;
}

const char* SimUtils::ProtocolPackage::getBuffer() const
{
  memcpy((void *) buffer, (void *) &dataLength, sizeof(int));
  //std::cerr << "Will send a package with: " << getLength() << " bytes\n";
  return buffer;
}

int SimUtils::ProtocolPackage::getLength() const
{
  return dataLength + headerLength;
}


void SimUtils::ProtocolPackage::clear()
{
  dataLength = 0;
}

void SimUtils::ProtocolPackage::put(std::string s)
{
  int n=0;
  int ssize = (int)s.size();

  if ((int)(dataLength + headerLength + 1 + sizeof(int) + ssize) > (int)maxBufferLen)
    {
      std::cerr << "Buffer overrun, will not put string to sendbuffer.\n";
      return;
    }

  char *ptr = &buffer[dataLength+headerLength];
  char t = (char)((int)DTypeString);
  ptr[0]=t;
  n+=1;
  //memcpy(ptr, &t, sizeof(int));
  //n+=sizeof(int);
  memcpy(&ptr[n], &ssize, sizeof(int));
  n+=sizeof(int);
  memcpy(&ptr[n], s.c_str(), ssize);
  n+=ssize;

  dataLength += n;
}

void SimUtils::ProtocolPackage::put(float v)
{
  int n=0;

  if ((int)(dataLength+headerLength+1+sizeof(float)) > maxBufferLen)
    {
      std::cerr << "Buffer overrun, will not put float to sendbuffer.\n";
      return;
    }

  char *ptr = &buffer[dataLength+headerLength];
  char t = (char) ((int)DTypeFloat);
  ptr[0]=t;
  n+=1;
  //int t = DTypeFloat;
  //memcpy(ptr, &t, sizeof(int));
  //n+=sizeof(int);
  memcpy(&ptr[n], &v, sizeof(float));
  n+=sizeof(float);

  dataLength += n;
}

void SimUtils::ProtocolPackage::put(const std::vector< float >& v)
{
  int n=0;
  int lsize = (int)v.size();

  char *ptr = &buffer[dataLength+headerLength];
  char t = (char) ((int)DTypeListOfFloat);
  ptr[0]=t;
  n+=1;

  memcpy(&ptr[n], &lsize, sizeof(int));
  n+=sizeof(int);

  for (int i=0; i< lsize; i++) {
    memcpy(&ptr[n], &v[i], sizeof(float));
    n+= sizeof(float);
  }

  dataLength += n;
}

void SimUtils::ProtocolPackage::put(char v)
{
  int n=0;
  
  if ((int)(dataLength+headerLength+sizeof(char)+sizeof(char)) > maxBufferLen)
    {
      std::cerr << "Buffer overrun, will not put char to sendbuffer.\n";
      return;
    }
  
  char *ptr = &buffer[dataLength+headerLength];
  /*   char t = (char) ((int)DTypeChar);
     ptr[0]=t;
     n+=1;
     ptr[n]=v;
     n+=1; */

  int t = DTypeChar;
  memcpy(ptr, &t, sizeof(int));
  n+=sizeof(int);
  memcpy(&ptr[n], &v, sizeof(char));
  n+=sizeof(char);

  dataLength += n;
}

void SimUtils::ProtocolPackage::startEval()
{
  evalCtr = 0;
}

bool SimUtils::ProtocolPackage::nextEvalString(std::string &vs)
{
  int n=0;
  char t=buffer[headerLength+evalCtr];
  n+=1;

  //int t;
  //memcpy(&t, &buffer[headerLength+evalCtr], sizeof(int));
  //n+=sizeof(int);
  if (DataTypeTag((int)t) != DTypeString) return false;
  int ssize;
  memcpy(&ssize, &buffer[headerLength+evalCtr+n], sizeof(int));
  n+=sizeof(int); 
  char tmp = buffer[headerLength+evalCtr+n+ssize];
  buffer[headerLength+evalCtr+n+ssize]='\0';
  vs = &buffer[headerLength+evalCtr+n];
  buffer[headerLength+evalCtr+n+ssize]=tmp;
  n+=ssize;
  evalCtr+=n;
  return true;
}

bool SimUtils::ProtocolPackage::nextEvalFloat(float &v)
{
  int n=0;
  char t = buffer[headerLength+evalCtr];
  n+=1;
  if (DataTypeTag((int)t) != DTypeFloat) return false;
  //int t;
  //memcpy(&t, &buffer[headerLength+evalCtr], sizeof(int));
  //n+=sizeof(int);
  //if (DataTypeTag(t) != DTypeFloat) return false;
  memcpy(&v, &buffer[headerLength+evalCtr+n], sizeof(float));
  n+=sizeof(float);
  evalCtr+=n;
  return true;
}

bool SimUtils::ProtocolPackage::nextEvalChar(char &v)
{
  int n=0;
  /*char t = buffer[headerLength+evalCtr];
    n+=1;
    if (DataTypeTag((int)t) != DTypeChar)  {
    std::cerr << "Received Type: " << t << "\n";
    return false;
    }*/
  int t;
  memcpy(&t, &buffer[headerLength+evalCtr], sizeof(int));
  n+=sizeof(int);
  if (DataTypeTag(t) != DTypeChar) {
    std::cerr << "Received Type: " << t << "\n";
    return false;
  }
  
  v = buffer[headerLength+evalCtr+n];
  //memcpy(&v, &buffer[headerLength+evalCtr+n], sizeof(char));
  n+=sizeof(char);
  evalCtr+=n;
  return true;
}


bool SimUtils::ProtocolPackage::nextEvalListOfFloat(std::vector< float >& v)
{
  v.clear();
  int n=0;
  char t=buffer[headerLength+evalCtr];
  n+=1;
 
  if (DataTypeTag((int)t) != DTypeListOfFloat) return false;
  int lsize;
  memcpy(&lsize, &buffer[headerLength+evalCtr+n], sizeof(int));
  n+=sizeof(int);
 
  for (int i=0; i<lsize; i++) {
    float dum;
    memcpy(&dum, &buffer[headerLength+evalCtr+n], sizeof(float));
    v.push_back(dum);
    n+=sizeof(float);
  }
  evalCtr+=n;
  return true;
}


SimUtils::SimSocket::SimSocket(const std::string &foreignAddress, 
				       unsigned short foreignPort) throw(SimUtils::SocketException) 
  : TCPSocket(foreignAddress,foreignPort)
{
  ;
}


SimUtils::SimSocket::SimSocket(int newSocketDesc)
  : TCPSocket(newSocketDesc) 
{
  ;
}

bool SimUtils::SimSocket::send(const ProtocolPackage  &pckg)  throw(SimUtils::SocketException)
{
 
  TCPSocket::send(pckg.getBuffer(), pckg.getLength());
  
  return true;
}

bool SimUtils::SimSocket::receive(ProtocolPackage &pckg) throw(SimUtils::SocketException)
{
  pckg.clear();

  int soll_packlen;
  int recv_packlen=0;
  
  int res=0;
  if ( (res=recv((void *) &soll_packlen, sizeof(int))) != sizeof(int) )
    {
      std::cerr << "Received not the package len! " << res << " \n";
      return false;
    }

  recv_packlen+=sizeof(int);
  while (recv_packlen < soll_packlen) {
    int n=recv((void *)&pckg.buffer[pckg.headerLength+pckg.dataLength],

	       MIN( pckg.maxBufferLen - (pckg.headerLength+pckg.dataLength), soll_packlen) );
    recv_packlen+=n;
    pckg.dataLength+=n;
  }
  
  return true;
}


bool SimUtils::SimSocket::initHandshake(const std::string& serverName, 
					std::string& clientName) throw(SimUtils::SocketException)
{
  ProtocolPackage ps;

  ps.put(serverName);
  ps.put((float)SOCKVER);
  
  send(ps);

  ProtocolPackage pr;  
  receive(pr);

  pr.startEval();
  if (!pr.nextEvalString(clientName)) return false;
  float v;
  if (!pr.nextEvalFloat(v)) return false;
  
  std::cerr << "Received: " << clientName << " Version: " << v << "\n";

  if (v != SOCKVER) {
    std::cerr << "Warning: The socket version does not match ... \n";
  }

  return true;
}

bool SimUtils::SimSocket::waitForHandshake(const std::string& clientName, 
					   std::string& serverName) throw(SimUtils::SocketException)
{
  ProtocolPackage pr;  
  receive(pr);
  
  pr.startEval();
  if (!pr.nextEvalString(serverName)) return false;
  float v;
  if (!pr.nextEvalFloat(v)) return false;
  
  std::cerr << "Received: " << serverName << " Version: " << v << "\n";
  
  if (v != SOCKVER) {
    std::cerr << "Warning: The socket version does not match ... \n";
  }

  ProtocolPackage ps;

  ps.put(clientName);
  ps.put((float)SOCKVER);
  
  send(ps);
  
  return true;
}
