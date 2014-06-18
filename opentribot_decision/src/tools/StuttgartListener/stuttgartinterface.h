#ifndef STUTTGARTINTERFACE
#define STUTTGARTINTERFACE

#include "tcpsocket.h"
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <sstream>



/* TIME STUFF *********************************** */

#include <math.h>
#include <sys/time.h>

/* TIME STUFF *********************************** */




using namespace std;

#define MAXMESG 10000

#define timeoutnum 500


long timeval2ms( timeval t ) {  
  long ret=0;
  ret=t.tv_sec*1000+(int)rint(t.tv_usec/1000.0);
  return ret;
}

long getTimestamp() {
  timeval timestamp;
  gettimeofday(&timestamp,NULL);
  return timeval2ms(timestamp);
}


struct WoMoData{
string refboxcommand;
int ballposx;
int ballposy;
bool gotgoto;
int gotox;
int gotoy;
int theta;
int stuttgartseesball;
int stuttgartballx;
int stuttgartbally;
int stuttgarthasball;

    bool ihavetheball;

WoMoData(){
gotgoto=0;
ballposx=0;
    gotgoto = 0;
    ballposx = 0;
ballposy=0;
stuttgarthasball=0;
stuttgartseesball=0;
ihavetheball=0;}


};


struct StuttgartMessage{
string msg; 

StuttgartMessage(){};

int constructStringMessage(string ToSend){

ostringstream ostr;
// A= adressiert B=Broadcast
ostr<<"MSG::SM::1::B::TRIBOT::TribotMessage::N::"<<getTimestamp()<<"//"<<ToSend<<"//::END\n";

msg=ostr.str();

//cout << ostr.str()<<endl; //OK LÄUFT

}


};






class StuttgartInterface{

public:


  WoMoData womodata;
  bool debug;
  char buf[MAXMESG];
  string connstate;
int nothing_received;  
string hostname;
int port;
    TCPsocket sock;

StuttgartInterface(){
   connstate="disconnected";
 nothing_received=0; 
  hostname="129.69.216.72"; 
	port=20002;
	debug=0;
	};



int communicate(){
	
    //memset(buf,'\0',MAXMESG);

    if (debug)cout << connstate<<endl;

    if (connstate=="disconnected"){

        bool res= TCPutils::init_client(hostname.c_str(),port, sock);

	cout << "Result of init_client res" <<res<<endl;
	if (res==0)connstate="connecting"; 

    }      

    if (connstate=="connecting"){
	int res=sock.recv_msg(buf,MAXMESG);
        if (res <=0){
	nothing_received++;
	if (nothing_received>timeoutnum){
	sock.close();	
	connstate="disconnected";
	}	
	}
	else{
	 
	cout << "Received"<<res<<"("<<buf <<")"<< endl;
	nothing_received=0;
	if (strcmp(buf,"ACK::CONNECT1")){
	connstate="wait_connect2";
     	string mesg="IDS,TRIBOT,0,RW,0,1\n";
     	cout << "sending "<< mesg<<endl;  
     	sock.send_msg(mesg.c_str(),strlen(mesg.c_str()));

	}

	}
	};
    
    if (connstate=="wait_connect2"){
	int res=sock.recv_msg(buf,MAXMESG);
        if (res <=0){
	nothing_received++;
	if (nothing_received>timeoutnum){
	sock.close();	
	connstate="disconnected";
	}	
	
	}
	else{ 
	cout << buf << endl;
	nothing_received=0;
	if (strcmp(buf,"ACK::CONNECT2")){
	connstate="connected";
	}

	}

	};

    if (connstate=="connected"){
	if(debug)cout << "Receiving..."<<endl;
	int received_packets=0;
	int res=1;
	while (res>0)
	{
		
		res=sock.recv_msg(buf,MAXMESG);
		if (res>0)
		{
		 	received_packets++;
			nothing_received=0;
			if ((debug)&&res==2 && strcmp(buf,"A\n")!=0)
				cout << "HEARTBEAT"<<endl;
			if (res>2)this->disectmessage();
		
	
		}
	}
  	if (received_packets==0){
	if (debug)cout <<" \nNothing received" <<endl;
	nothing_received++;
	if (nothing_received>timeoutnum){
	connstate="disconnected";
	sock.close();
	}
 	}


	};
 


return 0;
}





void Tokenize(string str, string delim,  vector<string> &result) {
  int cutAt;
  while( (unsigned int)(cutAt = str.find_first_of(delim)) !=str.npos) {
    if(cutAt > 0) {
      result.push_back(str.substr(0,cutAt));
    }
    str = str.substr(cutAt+1);
  }
  if(str.length() > 0) {
    result.push_back(str);
  }
}


void PrintVector(vector<string> &result) {
  cout << "Tokens Count: " << result.size() << endl;  
  for (unsigned int i = 0; i < result.size(); i++) {
    cout << "# TOKEN " << i << ": " <<result.at(i) << endl;
  }
}



int disectmessage(){
  vector<string> result;
  vector<string> header;
  vector<string> body;
   
  if (debug)cout << "MEssage: " << buf<< endl;
  
  // Tokenize Message into Header, Body, Footer.
if(debug) cout << "HEADER, BODY, FOOTER"<<endl;
  Tokenize(string(buf), "//", result);
 if(debug) PrintVector(result);
  
  /* Test if it is a correct Message
     Header should start with "MSG", Footer should be equal to "::END" */ 
  if ( !((result.size()==3) ||(strncmp(result.at(0).c_str(), "MSG",3)==0) ||(strcmp(result.at(2).c_str(), "::END")==0))) {

    cout << "Malformed Message" <<buf<<"|"<< endl;
}
  else {  
 if(debug)   cout <<" Tokenize Header"<<endl;
    Tokenize(result.at(0), "::", header);
   if(debug) PrintVector(header);
    
    // Tokenize Body
   if(debug) cout <<" Tokenize Body"<<endl;
    Tokenize(result.at(1), ",", body);
  
if(debug)    PrintVector(body);

    if (header[1]=="PC"){
    womodata.refboxcommand=body[0];
   cout << "Refboxcommand:"<<body[0]<<endl;
	
     }

    if (header[1]==""){
    
    
    
    
    
    
    }

    if (header[1]=="SL" && header.size()>10 && header[10]=="TRIBOT" ){
    if (body[0]=="GO") {
		womodata.gotoy=atoi(body[1].c_str());
		womodata.gotox=-atoi(body[2].c_str());
		womodata.theta=atoi(body[3].c_str());
		womodata.gotgoto=true;
		}
    if (body.size()>3 &&body[3]=="ball"){

		womodata.stuttgartseesball=atoi(body[4].c_str());
		womodata.stuttgartbally=atoi(body[5].c_str());
		womodata.stuttgartballx=-atoi(body[6].c_str());
	}
    if (body.size()>7&& body[7]=="OD"){
	   	womodata.stuttgarthasball=atoi(body[8].c_str());
	}


	
     
     // Send The I have ball message if possessball is on 
    
     if(womodata.ihavetheball){
	womodata.ihavetheball=false;
	StuttgartMessage smessage;
        smessage.constructStringMessage("1");     
     	cout <<"schicke"<< smessage.msg.c_str()<<endl;
	sock.send_msg(smessage.msg.c_str(),strlen(smessage.msg.c_str()));
	
	
	}
  
}


       
     
 } 
  // Test TimeStamp
 if(debug) cout << "Current Timestamp: " << getTimestamp() << endl;

  
  
  



return 0;
}




  };


#endif
