
#ifndef Tribots_RemoteTune_h
#define Tribots_RemoteTune_h

#define TUNER

#ifndef TUNER
#define TUNABLE(name,value);
#define SETINTVAL(name,value);
#define GETINTVAL(name,value);
#endif


#ifdef TUNER

#define TUNABLE(name,value) Tribots::RemoteTune::getTheRemoteTune()->registerlv(name,value);
#define SETINTVAL(name,value) Tribots::RemoteTune::getTheRemoteTune()->set_pointer(name,value);
#define GETINTVAL(name,value) Tribots::RemoteTune::getTheRemoteTune()->get_pointer(name,value);



#include <stdexcept>
#include <string>
#include <vector>
#include <sstream>
#include "../Communication/UDPSocket.h"

namespace Tribots {

  /** Klasse RemoteTune zum Abändern von Variablen zur Laufzeit Kommuniziert wird
      mittels Strings. Funktionsweise: Auf robotcontrol wird ein Server eingerichtet, der mit einem externen Tunigprogramm kommuniziert. Zunaechst speichert der Server alle Aenderungen lokal, bis diese durch einen Aufruf von TUNABLE(.,.) uebernommen werden. RemoteTune veraendert Programmvariablen also nicht selbstaendig. */
  class RemoteTune {
    struct VariableValue {
      std::string name;
      double storedvalue;
    };
    struct VariablePointer {
      std::string name;
      int storedvalue;
    };
  public:
    static RemoteTune *getTheRemoteTune ();
  protected:
      RemoteTune ();
  private:
    static RemoteTune *_remotetune;

    UDPSocket socket;
    char* messagebuffer;  ///< ein Puffer fuer die Kommunikation
    unsigned int messagelen;  ///< Nachrichtenlaenge
    unsigned int bufferlen;  ///< maximale Laenge von messagebuffer
    unsigned int changewhich;   ///< Index des zu veraendernden Parameters in values
    bool changelocal;   ///< Server-seitig: 'c' empfange, warte auf Wert

    std::vector<VariableValue> values;
    std::vector<VariablePointer> pointers;

  public:
    /** Server init */
    void initServer (int port);
    /** Client init */
    void initClient (const char *adress, int port);
    /** KommandoZeilen Server-Prozess, eine Iteration */
    void server_process ();
    /**Kommandozeilen Client-Prozess, eine Iteration */
    void client_process ();

    /** Variable anmelden und abgleichen */
    void registerlv (const char* name, double* value);

    void set_pointer(const char* name, int pointer);
    void get_pointer(const char* name, int* pointer);

  private:
    /** den i-ten Parameterwert als string verschicken */
    void sendTunableText(unsigned int i);

  };

}

#endif
#endif
