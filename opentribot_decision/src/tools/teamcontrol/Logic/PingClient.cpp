
#include <cstdio>
#include <sys/wait.h>
#include <sys/time.h>
#include <iostream>
#include "../../../Fundamental/stringconvert.h"
#include "PingClient.h"
#include <signal.h>

using namespace std;
using namespace Tribots;
using namespace TribotsTools;


PingClient::PingClient () {
  inetaddress = "localhost";
  addressChanged=true;
  childPID=0;
  latestPingTime=-1;
}

PingClient::~PingClient () {
  if (childPID>0)
    kill (childPID, SIGTERM);
}

double PingClient::responseTime (const std::string& s) {
  string::size_type tpos = s.find ("time=", 0);
  if (tpos==string::npos)
    return -1;
  tpos+=5;
  unsigned epos=tpos+1;
  while (epos<s.length()) {
    if (s[epos]==' ' || s[epos]=='\t' || s[epos]=='\n')
      break;
    epos++;
  }
  double ret;
  if (!string2double (ret, s.substr (tpos, epos-tpos)))
    return -2;
  return ret;
}

void PingClient::setAddress (const std::string& s) {
  mutexAccessAddress.lock();
  if (s!=inetaddress || childPID<=0) {
    inetaddress=s;
    addressChanged=true;
  }
  mutexAccessAddress.unlock();
}

std::string PingClient::getAddress () {
  std::string addr="";
  if (childPID>0) {
    mutexAccessAddress.lock();
    addr=inetaddress;
    mutexAccessAddress.unlock();
  }
  return addr;
}

void PingClient::exit () {
  if (childPID>0) {
    kill (childPID, SIGTERM);
  }
  latestPingTime=-1;
  POSIXThread::exit ();
}

void PingClient::threadCanceled () {
  if (childPID>0) {
    kill (childPID, SIGTERM);
  }
  latestPingTime=-1;
}

double PingClient::getPingTime () throw () {
  double ret;
  mutexAccessPingTime.lock();
  ret=latestPingTime;
  mutexAccessPingTime.unlock();
  return ret;
}

void PingClient::main () {
  char buffer [1024];
  int len;
  while (true) {
    checkCancel ();
    if (addressChanged) {
      mutexAccessPingTime.lock();
      latestPingTime=-3;
      mutexAccessPingTime.unlock();
      if (childPID>0) {
        kill (childPID, SIGTERM);
//        cerr << "kill " << childPID << endl;
        childPID=0;
        addressChanged=false;
      }
    }
    mutexAccessAddress.lock();
    string inetaddressLocal = inetaddress;
    mutexAccessAddress.unlock();
    if (childPID<=0 && inetaddressLocal.length()>0) {
      if (pipe(pipeDescriptor)==0) {
        pid_t newpid = fork ();
        if (newpid==0) {
          // Child-Process
          close (pipeDescriptor[0]);
          dup2(pipeDescriptor[1], STDOUT_FILENO);
          close (pipeDescriptor[1]);
//          cerr << "starte ping " << inetaddressLocal << endl;
          execlp("ping", "ping", inetaddressLocal.c_str(), NULL);

          // wenn irgendein Fehler aufgetreten ist mit execlp
          while (true)
            sleep (1000);
        } else {
          //Parent-Process
          childPID=newpid;
          close (pipeDescriptor[1]);
        }
      }
    }

    if (childPID<=0) {
      sleep(1);
      continue;
    }
    fd_set fdset;
    FD_ZERO(&fdset);
    FD_SET(pipeDescriptor[0], &fdset);
    struct timeval timeout;
    timeout.tv_sec=8;
    timeout.tv_usec=0;
    select (pipeDescriptor[0]+1, &fdset, NULL, NULL, &timeout);
    if (FD_ISSET (pipeDescriptor[0], &fdset)) {
      len=read(pipeDescriptor[0], buffer, 1023);
    } else {
      len=0;
    }
    if (len==-1) {
      perror("read() failed");
      continue;  // read error
    }
    double latestPingTimeLocal = responseTime (string(buffer, len));
    if (len==0) {
      sleep (1);
      latestPingTimeLocal = -2;
    }
//    cerr << len << " " << childPID << ' ' << latestPingTimeLocal << '\n';
    mutexAccessPingTime.lock();
    if (latestPingTime*latestPingTimeLocal<0)
      latestPingTime=latestPingTimeLocal;
    else
      latestPingTime=0.8*latestPingTime+0.2*latestPingTimeLocal;
    mutexAccessPingTime.unlock();
  }
}



/*
int main (int argc, char** argv) {
  PingClient ping;
  ping.setAddress (argv[1]);
  ping.start();
  string s="";
  while (s!="q") {
    cin >> s;
    cout << "TIME: " << ping.getPingTime() << endl;
    if (s.length()>1) {
      ping.setAddress(s);
    }
  }
  return 0;
}
*/
