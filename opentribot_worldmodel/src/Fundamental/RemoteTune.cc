
#include "RemoteTune.h"
#include <iostream>
#include <sstream>
#include "cstring"
#include "stdlib.h"
#ifdef TUNER

using namespace std;
using namespace Tribots;

RemoteTune* RemoteTune::_remotetune = 0;

RemoteTune* RemoteTune::getTheRemoteTune () {
  if (_remotetune == 0)
    _remotetune = new RemoteTune;
  return _remotetune;
}

RemoteTune::RemoteTune () {
  bufferlen=500;
  messagebuffer=new char[bufferlen];
  changewhich=0;
  changelocal=false;
}

void RemoteTune::initClient (const char *address, int port) {
  socket.init_as_client (address, port);
  cout<<"Trying to connect "<< address <<endl;
}

void RemoteTune::initServer (int port) {
  // 3 verschiedene Ports ausprobieren
  if (!socket.init_as_server (port)){
    if (!socket.init_as_server(port+1)) {
      if(!socket.init_as_server(port+2)) {;}
    }
  }
}

void RemoteTune::sendTunableText (unsigned int i){
  if (i>=values.size())
    return;
  stringstream inout;
  inout << "|" << values[i].name << " #" << values[i].storedvalue << '\n';
  string line;
  getline (inout, line);
  socket.send (line.c_str(), line.length()+1);
}


void RemoteTune::server_process () {
  // ggf. Nachricht empfangen
  if (socket.receive (messagebuffer, messagelen, bufferlen) > 0) {
    std::string message = messagebuffer;
    cout <<"Received Message::"<<message<<"::"<<endl;

    // pruefen, welche Nachricht empfangen wurde und reagieren:
    for (unsigned int i=0; i<values.size(); i++) {
      // pruefen, ob Nachricht der Schluessel eines Parameters ist
      if (message==values[i].name) {
        changewhich = i;
        sendTunableText(i);
      }
    }

    if (message=="/") {
      // Client begruessen
      socket.send ("Hallo", 6);
    }

    if (message=="h") {
      // Hilfe zureucksenden
      std::string help=">Hilfe für Remotetuner \n>l List available Tunables\n/Message Send Control Message";
      socket.send (help.c_str(), help.length()+1);
    } else if (message=="l") {
      // Liste aller Parameter schicken
      for (unsigned int i=0; i<values.size(); i++) {
        sendTunableText(i);
      }
    } else if (message=="n") {
      // weiterschalten
      changewhich++;
      if (changewhich>=values.size())
        changewhich=0;
      sendTunableText(changewhich);
    } else if (message=="p") {
      if (changewhich>0)
        changewhich--;
      else if (values.size()>0)
        changewhich=values.size()-1;
      else
        changewhich=0;
      sendTunableText(changewhich);
    } else if (message=="c") {
      stringstream inout;
      inout << ">Changing Variable |" << values[changewhich].name << '\n';
      string line;
      getline (inout, line);
      socket.send (line.c_str(), line.length()+1);
      changelocal = true;
    } else if (changelocal) {
      values[changewhich].storedvalue = atof (message.c_str());
      stringstream inout;
      inout << ">Changed Variable |" << values[changewhich].name << " to Value #" << values[changewhich].storedvalue << '\n';
      string line;
      getline (inout, line);
      socket.send(line.c_str(),line.length()+1);
      changelocal = false;
    }

    if (message=="q") {
      exit (0);
    }
  }
}



void RemoteTune::client_process () {
  // ggf. Nachrichten empfangen und ausgeben:
  while (socket.receive (messagebuffer, messagelen, bufferlen) > 0) {
    cout << messagebuffer << endl;
  }
  cout << ">: ";
  string command;
  cin >> command;
  if (command=="q") {
    cout << "Quit!!" << endl;
    exit (0);
  }
  cout <<"--"<<command<<"--"<<endl;
  socket.send (command.c_str(), command.length()+1);
}

void RemoteTune::registerlv (const char* name, double* setvalue) {
  for (unsigned int i=0;  i<values.size(); i++) {
    if (values[i].name==name) {
      *setvalue = values[i].storedvalue;
      return;
    }
  }
  VariableValue vv;
  vv.name=name;
  vv.storedvalue=*setvalue;
  values.push_back (vv);
}

void RemoteTune::set_pointer (const char* name, int pointer) {
  for (unsigned int i=0; i<pointers.size(); i++) {
    if (pointers[i].name==name) {
      pointers[i].storedvalue=pointer;
      return;
    }
  }
  VariablePointer vp;
  vp.name=name;
  vp.storedvalue=pointer;
  pointers.push_back (vp);
}

void RemoteTune::get_pointer (const char* name, int* value) {
  for (unsigned int i=0; i<pointers.size(); i++) {
    if (pointers[i].name==name) {
      *value= pointers[i].storedvalue;
    }
  }
}

#endif
