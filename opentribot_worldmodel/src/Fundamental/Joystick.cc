
#include "Joystick.h"
#ifndef __APPLE__
// schneller hack, richtige loesung fehlt noch...
#include <linux/joystick.h>
#endif
#include <fcntl.h>

#include <iostream>

using namespace std;

namespace {
  const double MAX_AXIS_VALUE = 32768;   // Groesste Auslenkung der Achsen
}

Tribots::Joystick::Joystick (const char* devname) throw (std::invalid_argument, std::bad_alloc) : axis (6), button (12) {
#ifndef __APPLE__
  file_descriptor = open(devname, O_RDONLY);
  if (file_descriptor<0)
    throw std::invalid_argument ("invalid device name in Tribots::Joystick::Joystick");
  
  fcntl(file_descriptor, F_SETFL, O_NONBLOCK); //don't block
  unsigned char n;
  ioctl(file_descriptor, JSIOCGAXES, &n);  // Anzahl Achsen abfragen
  axis.resize (n);
  ioctl(file_descriptor, JSIOCGBUTTONS, &n);  // Anzahl Knoepfe abfragen
  button.resize (n);
#endif
}

Tribots::Joystick::~Joystick () throw () {
#ifndef __APPLE__
  close (file_descriptor);
#endif
}

void Tribots::Joystick::update () throw () {
#ifndef __APPLE__
  js_event ev;
  int evsize = sizeof (struct js_event);
  while (true) {
    int size = read (file_descriptor, &ev, evsize);
    if (size!=evsize)
      break;

    if (ev.type==JS_EVENT_BUTTON)
      button[ev.number]=(ev.value==1);
    else if (ev.type==JS_EVENT_AXIS)
      axis[ev.number]=static_cast<double>(ev.value)/MAX_AXIS_VALUE;
  }
#endif
}

const std::vector<double>& Tribots::Joystick::get_axis_state () throw () {
  update ();
  return axis;
}

const std::vector<bool>& Tribots::Joystick::get_button_state () throw () {
  update ();
  return button;
}

std::string Tribots::Joystick::get_type () throw (std::bad_alloc) {
#ifndef __APPLE__
  char buffer [500];
  ioctl(file_descriptor, JSIOCGNAME(500), buffer);
  return std::string (buffer);
#endif
}

int Tribots::Joystick::get_version () throw () {
#ifndef __APPLE__
  int n;
  ioctl(file_descriptor, JSIOCGVERSION, &n);
  return n;
#endif
}

