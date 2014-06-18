#include <string.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>

using namespace std;


/* TIME STUFF *********************************** */

#include <math.h>
#include <sys/time.h>

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

/* TIME STUFF *********************************** */


void PrintVector(vector<string> &result) {
  cout << "Tokens Count: " << result.size() << endl;  
  cout << "---------------------------------------------" << endl;
  for (int i = 0; i < result.size(); i++) {
    cout << "# TOKEN " << i << ": " <<result.at(i) << endl;
  }
  cout << "---------------------------------------------" << endl;
}

void Tokenize(string str, string delim,  vector<string> &result) {
  int cutAt;
  while( (cutAt = str.find_first_of(delim)) != str.npos) {
    if(cutAt > 0) {
      result.push_back(str.substr(0,cutAt));
    }
    str = str.substr(cutAt+1);
  }
  if(str.length() > 0) {
    result.push_back(str);
  }
}


int main(int argc, char** arrv) {
  
  // Test WoMo-Message
  string contents = "MSG::WM::1::B::rfcbot1::System_WoMo_Data::N::-1267039682//12,INT,-1267039691,4,1,0,0,0,0,INT,-1267039682,4,1,-6877,-164,52,0,INT,-1267039691,7,1,1,-4358,3874,1568,1649,650,2795,INT,-1267039691,9,0,INT,-1267039718,2,0,INT,-1267039718,6,1,1,-4196,3486,1166,1878,0,INT,-1267039682,3,1,16,-16,-8,INT,-1267039682,7,1,-6877,-164,52,0,-6877,-164,52,INT,-31985088,3,0,INT,0,5,0,INT,-1267039682,3,0,STRING,-1267039682,10,1,wait,0,RefereeEvents,K,O,Stop,0,1,N,0//::END";
  
//string contents = "MSG::SL::1::A::rfcbot2::TribotMessage::N::-569609890::-::-::TRIBOT//GO,-4800,1,0//::END"; 
  
  
  // Test RefBox-Message stop
  //string contents = "MSG::PC::1::B::RefBoxCommandTester::System_Player_Commands::I::-1273146465//gameInterrupt//::END";
  
  
  // Test RefBox-Message start
  // string contents = "MSG::PC::1::B::RefBoxCommandTester::System_Player_Commands::I::-1273124738//go//::END";
  
  vector<string> result;
  vector<string> header;
  vector<string> body;
  
  cout << endl << endl << "Tokenize: " << contents << endl << endl;
  cout << "---------------------------------------------" << endl;
  
  // Tokenize Message into Header, Body, Footer.
  Tokenize(contents, "//", result);
  PrintVector(result);
  
  /* Test if it is a correct Message
     Header should start with "MSG", Footer should be equal to "::END" */ 
  if ( !(result.size()==3
	 && strncmp(result.at(0).c_str(), "MSG",3)==0
	 && strcmp(result.at(2).c_str(),"::END")==0)  ) {
    // Error in Message
    cout << "Malformed Message" << endl;

  } else {
    
    // Tokenize Header
    Tokenize(result.at(0), "::", header);
    cout << "timestamp:"<<header[7]<<endl;
    PrintVector(header);
    
    // Tokenize Body
    Tokenize(result.at(1), ",", body);
    PrintVector(body);
  }
  
  // Test TimeStamp
  cout << "Current Timestamp: " << getTimestamp() << endl;
  
  return 0;
}

