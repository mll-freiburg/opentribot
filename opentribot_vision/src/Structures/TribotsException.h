
#ifndef tribots_tribots_exception_h
#define tribots_tribots_exception_h

#include <exception>
#include <string>
#include <vector>

namespace Tribots {

  /** Ausnahme fuer Fehler der Tribotsanwednung, allgemein */
  class TribotsException : public std::exception {
  protected:
    std::string what_string;    ///< der string, der bei Aufruf von what() zurueckgeliefert wird
    void* btrace [50];            ///< Array fuer Backtrace
    int btrace_size;   ///< Groesse von 'backtrace'
    std::vector<std::string> btrace_text;   ///< textuelle Darstellung des Backtraces
  public:
    /** Konstruktor; Arg1: Beschreibung des Fehlers */
    TribotsException (const char*);
    ~TribotsException () throw () {;}

    /** liefere Fehlerbeschreibung */
    virtual const char* what () throw();
    
    /** liefere den Backtrace fuer den Zeitpunkt des Konstruktoraufrufs und schreibe ihn in den Stream arg1 */
    virtual const std::vector<std::string>& backtrace () throw ();
  };

  /** Ausnahme fuer Hardwarefehler, der eventuell bei einem weiteren Versuch
   *  mit dem gleichen Aufruf nicht wieder auftritt. Zum Beispiel: Ausfall
   *  der Kamera (repariert sich selbst), Ausfall der WLAN-Verbindung.
   */
  class HardwareException : public TribotsException {
  protected:
    std::string config_argument;   ///< die fehlerhafte Konfigurationsangabe
  public:    
    /** Konstruktor; Arg1: Bezeichnung fuer fehlende Konfigurationszeile */
    HardwareException (const char*);
    ~HardwareException () throw () {;}
  };

  /** Ausnahme fuer irreparable Fehler der Hardware, die einen Weiterbetrieb des Roboters nicht erlauben,
      z.B. Ausfall von Motoren, Ausfall der Board-Kommunikation, Absturz der Kamera */
  class BadHardwareException : public TribotsException {
  public:
    /** Konstruktor; Arg1: Beschreibung des Fehlers */    
    BadHardwareException (const char*);
    ~BadHardwareException () throw () {;}
  };
  
  /** Ausnahme fuer Fehler aufgrund fehlerhafter Konfigurations-Angaben */
  class InvalidConfigurationException : public TribotsException {
  protected:
    std::string config_argument;   ///< die fehlerhafte Konfigurationsangabe
  public:    
    /** Konstruktor; Arg1: Bezeichnung fuer fehlende Konfigurationszeile */
    InvalidConfigurationException (const char*);
    ~InvalidConfigurationException () throw () {;}
  };



  // hier koennen weitere benoetigte Ausnahmen eingefuegt werden

}

#endif

