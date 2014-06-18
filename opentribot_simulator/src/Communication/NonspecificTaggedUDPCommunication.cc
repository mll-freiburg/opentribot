
#include "NonspecificTaggedUDPCommunication.h"

using namespace Tribots;


NonspecificTaggedUDPCommunication::NonspecificTaggedUDPCommunication (const char* magic_bytes, unsigned int maxbufsize) throw (std::bad_alloc) : socket(magic_bytes, maxbufsize) {;}

NonspecificTaggedUDPCommunication::~NonspecificTaggedUDPCommunication () throw () {
  close ();
}

bool NonspecificTaggedUDPCommunication::init_as_server (int port) throw () {
  return socket.init_as_server (port);
}

bool NonspecificTaggedUDPCommunication::init_as_client (const char* host, int port) throw () {
  return socket.init_as_client (host, port);
}

void NonspecificTaggedUDPCommunication::close () throw () {
  socket.close();
}

unsigned int NonspecificTaggedUDPCommunication::max_message_length () const throw () {
  return socket.max_message_length();
}

const struct sockaddr_in& NonspecificTaggedUDPCommunication::partner_address () const throw () {
  return socket.partner_address();
}

bool NonspecificTaggedUDPCommunication::started() const throw () {
  return socket.started();
}

bool NonspecificTaggedUDPCommunication::as_server() const throw () {
  return socket.as_server();
}

unsigned int NonspecificTaggedUDPCommunication::send () throw () {
  return socket.send();
}

unsigned int NonspecificTaggedUDPCommunication::sendto (const struct sockaddr_in& add) throw () {
  return socket.sendto(add);
}

unsigned int NonspecificTaggedUDPCommunication::receive () throw (std::bad_alloc) {
  return socket.receive();
}

unsigned int NonspecificTaggedUDPCommunication::receivefrom (const struct sockaddr_in& add) throw (std::bad_alloc) {
  return socket.receivefrom (add);
}

void NonspecificTaggedUDPCommunication::clear_send_buffer () throw () {
  socket.clear_send_buffer();
}

void NonspecificTaggedUDPCommunication::clear_receive_buffer () throw () {
  socket.clear_receive_buffer();
}

UDPCommunicationStatistics NonspecificTaggedUDPCommunication::send_load (int dt) const throw () {
  return socket.send_load(dt);
}

UDPCommunicationStatistics NonspecificTaggedUDPCommunication::receive_load (int dt) const throw () {
  return socket.receive_load(dt);
}



void NonspecificTaggedUDPCommunication::write_signed_short (char* buffer, signed short int val) throw () {
  write_unsigned_short (buffer, static_cast<unsigned short int>(val));
}

signed short int NonspecificTaggedUDPCommunication::read_signed_short (const char* buffer) const throw () {
  return static_cast<signed int>(read_unsigned_short (buffer));
}

void NonspecificTaggedUDPCommunication::write_signed_short (char* buffer, double val) throw () {
  write_signed_short (buffer, static_cast<signed short int>(val));
}

void NonspecificTaggedUDPCommunication::write_unsigned_short (char* buffer, unsigned short int val) throw () {
  buffer[0]=static_cast<unsigned char>(val%256);
  buffer[1]=static_cast<unsigned char>(val/256);
}

unsigned short int NonspecificTaggedUDPCommunication::read_unsigned_short (const char* buffer) const throw () {
  return static_cast<unsigned short int>(static_cast<unsigned char>(buffer[0]))+(static_cast<unsigned short int>(static_cast<unsigned char>(buffer[1]))<<8);
}

void NonspecificTaggedUDPCommunication::write_unsigned_short (char* buffer, double val) throw () {
  write_unsigned_short (buffer, static_cast<unsigned short int>(val));
}
