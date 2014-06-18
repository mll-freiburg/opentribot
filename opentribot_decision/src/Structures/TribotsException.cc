#include "stdlib.h"
#include "TribotsException.h"
#ifndef __APPLE__
#include <execinfo.h>
#endif

Tribots::TribotsException::TribotsException (const char* err_type) {
#ifndef __APPLE__
  what_string = std::string ("TribotsException: ")+std::string(err_type);
  btrace_size = ::backtrace (btrace, 50);
#endif
}

const char* Tribots::TribotsException::what () throw () {
  return what_string.c_str();
}

const std::vector<std::string>& Tribots::TribotsException::backtrace () throw () {
#ifndef __APPLE__
  if (btrace_text.size()==0) {
    btrace_text.resize (btrace_size);
    char** btrace_symbols = ::backtrace_symbols(btrace, btrace_size);
    for (int i=0; i<btrace_size; i++)
      btrace_text[i] = btrace_symbols[i];
    free (btrace_symbols);
  }
#endif
  return btrace_text;
}


Tribots::HardwareException::HardwareException (const char* err_type) : TribotsException (err_type) {;}

Tribots::BadHardwareException::BadHardwareException (const char* err_type) : TribotsException (err_type) {;}

Tribots::InvalidConfigurationException::InvalidConfigurationException (const char* err_line) : TribotsException ((std::string ("invalid configuration line \"")+std::string(err_line)+std::string("\"")).c_str()) {;}

