
#include "TaggedUDPCommunication.h"

#include <cstring>

using namespace Tribots;
using namespace std;

TaggedUDPCommunication::TaggedUDPCommunication (const char* mb, unsigned int ml) throw (std::bad_alloc) : socket (mb, ml+1), received_messages (256), received_message_length (256) {;}


TaggedUDPCommunication::~TaggedUDPCommunication () throw () {;}


unsigned int TaggedUDPCommunication::max_message_length () const throw () {
  return socket.max_message_length()-1;
}

bool TaggedUDPCommunication::put (unsigned char tag, const char* msg, unsigned int msg_len, unsigned int prio) throw (std::bad_alloc) {
  char* interbuf = new char [msg_len+1];
  interbuf[0]=static_cast<char>(tag);
  memcpy (interbuf+1, msg, msg_len);
  bool success=socket.put (interbuf, msg_len+1, prio);
  delete [] interbuf;
  return success;
}

unsigned int TaggedUDPCommunication::num_messages (unsigned char tag) throw () {
  return received_messages[tag].size();
}

bool TaggedUDPCommunication::get (unsigned char tag, const char*& msg, unsigned int& msg_len) throw () {
  return get (tag, msg, msg_len, num_messages(tag)-1);
}

bool TaggedUDPCommunication::get (unsigned char tag, const char*& msg, unsigned int& msg_len, unsigned int index) throw () {
  if (index<received_messages[tag].size()) {
    msg = received_messages[tag][index];
    msg_len = received_message_length[tag][index];
    return true;
  }
  msg_len=0;
  msg=NULL;
  return false;
}


unsigned int TaggedUDPCommunication::send () throw () {
  return socket.send();
}

unsigned int TaggedUDPCommunication::sendto (const struct sockaddr_in& add) throw () {
  return socket.sendto(add);
}

unsigned int TaggedUDPCommunication::receive () throw (std::bad_alloc) {
  return receive_intern (NULL);
}

unsigned int TaggedUDPCommunication::receivefrom (const struct sockaddr_in& add) throw (std::bad_alloc) {
  return receive_intern (&add);
}

unsigned int TaggedUDPCommunication::receive_intern (const struct sockaddr_in* addp) throw (std::bad_alloc) {
  // alte Werte wegwerfen
  for (unsigned int i=0; i<256; i++) {
    received_messages[i].clear();
    received_message_length[i].clear();
  }

  // Socket auslesen
  unsigned int num_packets;
  if (addp)
    num_packets=socket.receivefrom(*addp);
  else
    num_packets=socket.receive ();

  // Nachrichten nach Tag sortieren
  const char* msg;
  unsigned int msg_len;
  bool next = socket.get (msg, msg_len);
  while (next) {
    if (msg_len>0) {  // ganz leere Nachrichten ohne Tag ignorieren
      unsigned char tag = static_cast<unsigned char>(msg[0]);
      received_messages[tag].push_back (msg+1);
      received_message_length[tag].push_back (msg_len-1);
    }
    next = socket.get (msg, msg_len);
  }

  return num_packets;
}


bool TaggedUDPCommunication::init_as_server (int port) throw () {
  return socket.init_as_server (port);
}

bool TaggedUDPCommunication::init_as_client (const char* host, int port) throw () {
  return socket.init_as_client (host, port);
}

void TaggedUDPCommunication::close () throw () {
  socket.close();
}

const struct sockaddr_in& TaggedUDPCommunication::partner_address () const throw () {
  return socket.partner_address();
}

bool TaggedUDPCommunication::started() const throw () {
  return socket.started();
}

bool TaggedUDPCommunication::as_server() const throw () {
  return socket.as_server();
}

void TaggedUDPCommunication::clear_send_buffer () throw () {
  socket.clear_send_buffer();
}

void TaggedUDPCommunication::clear_receive_buffer () throw () {
  socket.clear_receive_buffer();
  for (unsigned int i=0; i<256; i++) {
    received_messages[i].clear();
    received_message_length[i].clear();
  }
}

UDPCommunicationStatistics TaggedUDPCommunication::send_load (int dt) const throw () {
  return socket.send_load(dt);
}

UDPCommunicationStatistics TaggedUDPCommunication::receive_load (int dt) const throw () {
  return socket.receive_load(dt);
}

