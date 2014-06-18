
#ifndef TribotsTools_RefboxClient_h
#define TribotsTools_RefboxClient_h

// angepasst aus und angelehnt an CTCPIP_Client, der mit der Refereebox mitgeliefert wird.

#include "../../../Structures/GameState.h"
#include <fstream>
#include <deque>
#include <vector>
#include <string>

namespace TribotsTools {

  /** Klasse, die sich mit der Refereebox verbinden kann und 
      Refereebox-Signale in Tribots::RefboxSignal umwandelt */
  class RefboxClient {
  public:
    struct RefboxMessage {
      Tribots::RefboxSignal signal;
      std::vector<std::string> messages;
    };
    
    /** Konstruktor, Name der Logdatei wird uebergeben */
    RefboxClient (const char* ="/dev/null") throw ();
    
    /** Destruktor, loest Verbindung zu Refereebox auf */
    ~RefboxClient () throw ();

    /** Verbinden mit der Refereebox;
        Arg1: IP-Adresse
        Arg2: Port
        Return: erfolgreich verbunden? */
    bool connect (const char*, int) throw ();
    /** Verbindung aufloesen */
    void disconnect () throw ();

    /** pruefen, ob Informationen eingegangen sind und ggf. umsetzen in RefboxSignals 
        liegen keine Nahrichten vor, wird SIGnop zurueckgeliefert */
    RefboxMessage listen () throw ();

    /** abfragen, ob Verbindung okay */
    bool is_okay () const throw ();
    
    /** abfragen, ob Verbindung hergestellt wurde */
    bool is_connected () const throw ();
    
  private:
    Tribots::RefboxSignal latest_signal;    ///< letztes Signal, das RefereeState veraendert hat (um SIGstart und SIGready unterscheiden zu koennen)
    int sockfd;                             ///< Socket -Filedescriptor
    int okayfailed;                         ///< zaehlt, wie oft select !=0 geliefert hat
    bool connected;                         ///< true, wenn Verbindung hergestellt wurde
    std::ofstream logstream;                ///< Logdatei
    std::deque<char> unresolved_signals;    ///< noch nicht verarbeitete Signale der Refereebox
    std::string welcomePrefix;               ///< der bereits angekommene Teil der Welcome-Message
    bool waitforwelcome;             ///< kommt eventuell noch eine Welcome-Message?
  };

}

#endif
