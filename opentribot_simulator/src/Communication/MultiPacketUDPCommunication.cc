
#include "MultiPacketUDPCommunication.h"

using namespace Tribots;
using namespace std;

MultiPacketUDPCommunication::MultiPacketUDPCommunication () throw () : receivePacketStatistics (5000), sendPacketStatistics (5000) {
  for (unsigned int i=0; i<receivePacketStatistics.size(); i++) {
    receivePacketStatistics.get().size=0;
    receivePacketStatistics.step();
  }
  for (unsigned int i=0; i<sendPacketStatistics.size(); i++) {
    sendPacketStatistics.get().size=0;
    sendPacketStatistics.step();
  }
}

MultiPacketUDPCommunication::~MultiPacketUDPCommunication () throw () {
  for (unsigned int i=0; i<receivedBuffer.size(); i++)
    for (unsigned int j=0; j<receivedBuffer[i].size(); j++)
      delete [] receivedBuffer[i][j];
}

bool MultiPacketUDPCommunication::init_as_server (int port) throw () {
  return socket.init_as_server (port);
}

bool MultiPacketUDPCommunication::init_as_client (const char* host, int port) throw () {
  bool success = socket.init_as_client (host, port);
  partnerAddressMulti = socket.partner_address();
  return success;
}

void MultiPacketUDPCommunication::close () throw () {
  socket.close ();
}

unsigned int MultiPacketUDPCommunication::send (const std::vector<char*> buffers, const std::vector<unsigned int> buflens) throw () {
  unsigned num_success_send=0;
  unsigned int num_packets = buffers.size();
  if (buflens.size()<buffers.size())
    num_packets=buflens.size();
  for (unsigned int i=0; i<num_packets; i++) {
    if (socket.sendto (partnerAddressMulti, buffers[i], buflens[i])) {
      num_success_send++;
      sendPacketStatistics.get().size=buflens[i];
      sendPacketStatistics.get().timestamp.update();
      sendPacketStatistics.get().partner_address=partnerAddressMulti;
      sendPacketStatistics.step();
    }
  }
  return num_success_send;
}

unsigned int MultiPacketUDPCommunication::sendto (const struct sockaddr_in& add, const std::vector<char*> buffers, const std::vector<unsigned int> buflens) throw () {
  partnerAddressMulti = add;
  return send (buffers, buflens);
}

void MultiPacketUDPCommunication::receive_all (unsigned int max_buf_len) throw (std::bad_alloc) {
  char* buf = new char [max_buf_len];
  unsigned int len=0;
  while (true) {
    bool success = socket.receive (buf, len, max_buf_len);
    if (success) {
      int sender_index=-1;
      for (unsigned int i=0; i<senderAddresses.size(); i++)
        if (senderAddresses[i]==socket.partner_address ()) {
          sender_index=i;
          break;
        }
        if (sender_index==-1) {
          senderAddresses.push_back (socket.partner_address ());
          vector<char*> ncb;
          vector<unsigned int> nbl;
          receivedBuffer.push_back (ncb);
          receivedBufferLength.push_back (nbl);
          sender_index = senderAddresses.size()-1;
        }
        receivedBuffer[sender_index].push_back (buf);
        receivedBufferLength[sender_index].push_back (len);
        buf = new char [max_buf_len];
        receivePacketStatistics.get().size=len;
        receivePacketStatistics.get().timestamp.update();
        receivePacketStatistics.get().partner_address=socket.partner_address ();
        receivePacketStatistics.step();
    } else {
      break;
    }
  }
  delete [] buf;
}

unsigned int MultiPacketUDPCommunication::receivefrom (const struct sockaddr_in& add, std::vector<char*>& buffers, std::vector<unsigned int>& buflens, unsigned int max_buf_len) throw (std::bad_alloc) {
  receive_all (max_buf_len);

  // nach den Paketen des angefragten Kommunikationspartners sehen
  buffers.clear();
  buflens.clear();
  int request_index=-1;
  for (unsigned int i=0; i<senderAddresses.size(); i++)
    if (add==senderAddresses[i]) {
      request_index=i;
      break;
    }
  if (request_index==-1) {
    return 0;
  }
  for (unsigned int i=0; i<receivedBuffer[request_index].size(); i++) {
    buffers.push_back (receivedBuffer[request_index][i]);
    buflens.push_back (receivedBufferLength[request_index][i]);
  }
  senderAddresses.erase (senderAddresses.begin()+request_index);
  receivedBuffer.erase (receivedBuffer.begin()+request_index);
  receivedBufferLength.erase (receivedBufferLength.begin()+request_index);
  partnerAddressMulti = add;
  return buffers.size();
}

unsigned int MultiPacketUDPCommunication::receive (std::vector<char*>& buffers, std::vector<unsigned int>& buflens, unsigned int max_buf_len) throw (std::bad_alloc) {
  receive_all (max_buf_len);
  buffers.clear();
  buflens.clear();
  if (senderAddresses.size()==0)
    return 0;
  for (unsigned int i=0; i<receivedBuffer[0].size(); i++) {
    buffers.push_back (receivedBuffer[0][i]);
    buflens.push_back (receivedBufferLength[0][i]);
  }
  partnerAddressMulti = senderAddresses[0];
  senderAddresses.erase (senderAddresses.begin());
  receivedBuffer.erase (receivedBuffer.begin());
  receivedBufferLength.erase (receivedBufferLength.begin());
  return buffers.size();
}

const struct sockaddr_in& MultiPacketUDPCommunication::partner_address () const throw () {
  return partnerAddressMulti;
}

bool MultiPacketUDPCommunication::started() const throw () {
  return socket.started();
}

bool MultiPacketUDPCommunication::as_server() const throw () {
  return socket.as_server();
}

UDPCommunicationStatistics MultiPacketUDPCommunication::send_load (int dt) const throw () {
  UDPCommunicationStatistics ret;
  ret.packet_rate=0;
  ret.packet_size=0;
  Time from;
  Time latest;
  from.add_sec (-dt);
  for (int i=1; i<=static_cast<int>(sendPacketStatistics.size()); i++) {
    const UDPPacketStatistics& pk = sendPacketStatistics[-i];
    if ((latest=pk.timestamp)<from)
      break;
    if (pk.size>0) {
      ret.packet_rate++;
      ret.packet_size+=pk.size;
    }
  }
  if (ret.packet_rate>0) {
    ret.packet_size/=ret.packet_rate;
    if (latest<from)
      ret.packet_rate/=dt;
    else
      ret.packet_rate/=(1e-3*latest.elapsed_msec());
  }
  return ret;
}

UDPCommunicationStatistics MultiPacketUDPCommunication::receive_load (int dt) const throw () {
  UDPCommunicationStatistics ret;
  ret.packet_rate=0;
  ret.packet_size=0;
  Time from;
  Time latest;
  from.add_sec (-dt);
  for (int i=1; i<=static_cast<int>(receivePacketStatistics.size()); i++) {
    const UDPPacketStatistics& pk = receivePacketStatistics[-i];
    if ((latest=pk.timestamp)<from)
      break;
    if (pk.size>0) {
      ret.packet_rate++;
      ret.packet_size+=pk.size;
    }
  }
  if (ret.packet_rate>0) {
    ret.packet_size/=ret.packet_rate;
    if (latest<from)
      ret.packet_rate/=dt;
    else
      ret.packet_rate/=(1e-3*latest.elapsed_msec());
  }
  return ret;
}
