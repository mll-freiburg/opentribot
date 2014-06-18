
#ifndef Tribots_MessageBoard_h
#define Tribots_MessageBoard_h

#include <stdexcept>
#include <string>
#include <vector>
#include <sstream>

namespace Tribots {

  /** Klasse MessageBoard zum Austausch von Informationen zwischen Spielern und Trainer;
      dadurch auch indirekte Kommunikation zwischen Spielern moeglich. Kommuniziert wird
      mittles Strings. Wegen beschraenkter Bandbreite die Kommunikation kurz halten */
  class MessageBoard {
  public:
    /** Konstruktor */
    MessageBoard () throw (std::bad_alloc);
    /** Copy-Konstruktor */
    MessageBoard (const MessageBoard&) throw (std::bad_alloc);
    const MessageBoard& operator= (const MessageBoard&) throw (std::bad_alloc);

    /** die ankommenden Mitteilungen lesen */
    const std::vector<std::string> get_incoming () const throw ();
    /** die ankommenden Mitteilungen nach Prefix durchsuchen, liefert erste entsprechende Zeile oder Leerstring */
    const std::string& scan_for_prefix (const std::string&) const throw ();
    /** die ankommenden Mitteilungen nach Prefix durchsuchen, liefert erste entsprechende Zeile oder Leerstring */
    const std::string& scan_for_prefix (const char*) const throw ();
    /** die ausgehenden Mitteilungen lesen */
    const std::vector<std::string> get_outgoing () const throw ();

    /** das MessageBoard loeschen, d.h. ein- und ausgehende Mittelungen entfernen */
    void clear () throw ();
    /** eingehende Nachrichten loeschen */
    void clear_incoming () throw ();
    /** ausgehende Nachrichten loeschen */
    void clear_outgoing () throw ();
    
    /** eine ausgehende Nachricht in das MessageBoard schreiben */
    void publish (const std::string&) throw ();
    /** mehrere ausgehende Nachrichten in das MessageBoard schreiben */
    void publish (const std::vector<std::string>&) throw ();
    /** eine ausgehende Nachricht in das MessageBoard schreiben */
    void publish (const char*) throw ();
    /** einen Ausgabeestream des MessageBoards fuer ausgehende Nachrichten bekommen;
	einzelne Zeilen muessen mit '\n' abgeschlossen werden */
    std::ostream& publish_stream () throw ();

    /** eine eingehende Nachricht in das MessageBoard schreiben */
    void receive (const std::string&) throw ();
    /** eine Liste eingehender Nachrichten in das MessageBoard schreiben */
    void receive (const std::vector<std::string>&) throw ();
    /** eine eingehende Nachricht in das MessageBoard schreiben */
    void receive (const char*) throw ();

  private:
    std::vector<std::string> incoming;
    std::vector<std::string> outgoing;
    std::stringstream outgoing_stream;
    const std::string null_string;

    void flush_outgoing ();
  };

}

#define MOUT Tribots::WorldModel::get_main_world_model().get_message_board().publish_stream()

#endif
