
#include "PriorityUDPCommunication.h"
#include <cstring>

using namespace Tribots;
using namespace std;


PriorityUDPCommunication::PriorityUDPCommunication (const char* mb, unsigned int ml) throw (std::bad_alloc) : max_buffer_size (ml+2), send_buffers (0), send_buffer_length (0), send_buffer_priorities (0), receive_buffers (0), receive_buffer_length (0), get_buffer (0), get_message (0) {
  magic_byte[0]=mb[0];
  magic_byte[1]=mb[1];
}


PriorityUDPCommunication::~PriorityUDPCommunication () throw () {
  clear_send_buffer ();
  clear_receive_buffer ();
}


void PriorityUDPCommunication::clear_receive_buffer () throw () {
  for (unsigned int i=0; i<receive_buffers.size(); i++)
    delete [] receive_buffers[i];
  receive_buffers.clear();
  receive_buffer_length.clear();
  get_buffer=0;
  get_message=0;
}


void PriorityUDPCommunication::clear_send_buffer () throw () {
  for (unsigned int i=0; i<send_buffers.size(); i++)
    delete [] send_buffers[i];
  send_buffers.clear();
  send_buffer_length.clear();
  send_buffer_priorities.clear();
}


unsigned int PriorityUDPCommunication::max_message_length () const throw () {
  return max_buffer_size-4;
}


unsigned int PriorityUDPCommunication::send () throw () {
  unsigned int res = socket.send (send_buffers, send_buffer_length);
  clear_send_buffer ();
  return res;
}


unsigned int PriorityUDPCommunication::sendto (const struct sockaddr_in& add) throw () {
  unsigned int res = socket.sendto (add, send_buffers, send_buffer_length);
  clear_send_buffer ();
  return res;
}


unsigned int PriorityUDPCommunication::receive () throw (std::bad_alloc) {
  clear_receive_buffer ();
  unsigned int res = socket.receive (receive_buffers, receive_buffer_length, max_buffer_size);
  return res;
}


unsigned int PriorityUDPCommunication::receivefrom (const struct sockaddr_in& add) throw (std::bad_alloc) {
  clear_receive_buffer ();
  unsigned int res = socket.receivefrom (add, receive_buffers, receive_buffer_length, max_buffer_size);
  return res;
}


bool PriorityUDPCommunication::put (const char* msg, unsigned int msg_len, unsigned int prio) throw (std::bad_alloc) {
  // Nachrichtenformat:  LHN oder LN, je nach Lanege der Nachricht.
  // N=Nutzdaten, d.h. *msg, L und H codieren die Nachrichtenlaenge.
  // fuer Nachrichten <128 Byte wird Format LN verwendet, wobei L die Nachrichtenlaenge angibt (1Byte)
  // fuer Nachrichten >=128 Byte und < 32768 Byte wird Format LHN verwendet, wobei:
  //   L Bits 0-6 codieren die Nachrichtenlaenge modulo 2^7
  //   L Bit 7 ist gesetzt
  //   H Bits 0-7 codieren die Nachrichtenlaenge geteilt 2^7
  // d.h. H und L-Bits0-6 codieren die Nachrichtenlaenge binaer, L Bit7 gibt an, ob Laenge mit ein oder zwei Bytes codiert wird

  if (msg_len+4>max_buffer_size)
    return false;   // Nachricht zu lang

  // Codierung der Nachrichtengroesse
  unsigned int len_code_len = (msg_len>127 ? 2 : 1);
  unsigned char low = 0, high = 0;
  if (len_code_len==1)
    low = static_cast<unsigned char>(msg_len);
  else {
    high = static_cast<unsigned char>(msg_len>>7);
    low = 128 | static_cast<unsigned char>(msg_len%128);
  }

  // freien Puffer passender Prioritaet suchen
  unsigned target_buffer=send_buffers.size();
  for (unsigned int i=0; i<send_buffers.size(); i++)
    if (send_buffer_priorities[i]==prio && send_buffer_length[i]+len_code_len+msg_len<=max_buffer_size) {
      target_buffer=i;
      break;
    }
  if (target_buffer==send_buffers.size()) {
    send_buffers.push_back (new char [max_buffer_size]);
    send_buffers[send_buffers.size()-1][0]=magic_byte[0];  // Pakete markieren
    send_buffers[send_buffers.size()-1][1]=magic_byte[1];
    send_buffer_length.push_back (2);
    send_buffer_priorities.push_back (prio);
  }

  // in Target buffer schreiben
  (*(send_buffers[target_buffer]+send_buffer_length[target_buffer]))=static_cast<char>(low);
  if (len_code_len==2)
    (*(send_buffers[target_buffer]+send_buffer_length[target_buffer]+1))=static_cast<char>(high);
  memcpy ((send_buffers[target_buffer]+send_buffer_length[target_buffer]+len_code_len), msg, msg_len);
  send_buffer_length[target_buffer]+=len_code_len+msg_len;
  return true;
}


bool PriorityUDPCommunication::get (const char*& mb, unsigned int& ml) throw () {
  if (get_buffer>=receive_buffers.size() || get_message>=receive_buffer_length[get_buffer]) {
    // keine Nachricht mehr da
    mb = NULL;
    ml = 0;
    return false;
  }

  if (get_message==0) {
    if (receive_buffer_length[get_buffer]<3) {
      // Paket enthaelt nicht mal die magic_bytes oder nur die magic_bytes
      get_buffer++;
      get_message=0;
      return get (mb, ml);  // im naechtsen Paket suchen
    }
    if (receive_buffers[get_buffer][0]!=magic_byte[0] || receive_buffers[get_buffer][1]!=magic_byte[1]) {
      // Paket nicht mit den magic_bytes markiert->nicht von PriorityUDPCommunication erzeugt->ueberspringe Paket
      get_buffer++;
      get_message=0;
      return get (mb, ml);  // im naechtsen Paket suchen
    }
    get_message=2;
  }

  // Nachrichtenlaenge ermitteln
  unsigned char low = static_cast<unsigned char>(*(receive_buffers[get_buffer]+get_message));
  if (low & 128) {
    // 2-Byte-Nachrichtenlaenge
    get_message++;
    if (get_message>=receive_buffer_length[get_buffer]) {
      // 2. Byte nicht gefunden; interpretiere Nachricht als "leer"
      get_buffer++;
      get_message=0;
      mb = NULL;
      ml = 0;
      return true;
    }
    unsigned int high = static_cast<unsigned char>(*(receive_buffers[get_buffer]+get_message));
    ml = (static_cast<unsigned int>(high)<<7)+static_cast<unsigned int>(low&127);
  } else {
    // 1-Byte-Nachrichtenlaege
    ml = static_cast<unsigned int>(low&127);
  }
  get_message++;

  // Nachricht lesen
  if (receive_buffer_length[get_buffer]-get_message>=ml) {
    mb=receive_buffers[get_buffer]+get_message;
    get_message+=ml;
    if (get_message>=receive_buffer_length[get_buffer]) {
      get_buffer++;
      get_message=0;
    }
    return true;
  } else {
    // Nachricht nicht vollstaendig im Puffer; interpretiere Nachricht als "leer"
    get_buffer++;
    get_message = 0;
    mb = NULL;
    ml = 0;
    return true;
  }
}

bool PriorityUDPCommunication::init_as_server (int port) throw () {
  return socket.init_as_server (port);
}

bool PriorityUDPCommunication::init_as_client (const char* host, int port) throw () {
  return socket.init_as_client (host, port);
}

void PriorityUDPCommunication::close () throw () {
  return socket.close();
}

const struct sockaddr_in& PriorityUDPCommunication::partner_address () const throw () {
  return socket.partner_address ();
}

bool PriorityUDPCommunication::started() const throw () {
  return socket.started();
}

bool PriorityUDPCommunication::as_server() const throw () {
  return socket.as_server();
}

UDPCommunicationStatistics PriorityUDPCommunication::send_load (int dt) const throw () {
  return socket.send_load(dt);
}

UDPCommunicationStatistics PriorityUDPCommunication::receive_load (int dt) const throw () {
  return socket.receive_load(dt);
}

