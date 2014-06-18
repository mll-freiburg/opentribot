#include <stdlib.h>
#include "Journal.h"
#include <iostream>
#include "TribotsException.h"

namespace {

  void default_terminate_handler () {
    Tribots::TribotsException dc (""); // nur wegen Backtrace erzeugt
    JERROR ("aborted due to an exception that wasn't caught in:");
    for (unsigned int i=5; i<dc.backtrace().size(); i++) // die ersten 5 Zeilen sind uninteressant
      JMESSAGE (dc.backtrace()[i].c_str());
    Tribots::Journal::the_journal.flush();
    std::cerr << "aborted due to an exception that wasn't caught;\n";
    std::cerr << "consult the journal for more information.\n";
    exit (-1);
  }
 
  void default_unexpected_handler () {
    Tribots::TribotsException dc (""); // nur wegen Backtrace erzeugt
    JERROR ("unexpected exception occured in:");
    for (unsigned int i=4; i<dc.backtrace().size(); i++) // die ersten 4 Zeilen sind uninteressant
      JMESSAGE (dc.backtrace()[i].c_str());
    Tribots::Journal::the_journal.flush();
    std::cerr << "unexpected exception occured in:\n";
    std::cerr << "consult the journal for more information.\n";
    exit (-1);
  }

  void (*old_terminate_handler)() = std::set_terminate(default_terminate_handler);
  void (*old_unexpected_handler)() = std::set_unexpected(default_unexpected_handler); 
     
}
