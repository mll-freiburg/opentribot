
#ifndef _Tribots_ConfigReader_h_
#define _Tribots_ConfigReader_h_

#include <vector>
#include <string>
#include <stdexcept>
#include <iostream>
#include <stack>

namespace Tribots {

  /** ConfigReader, Alternative zum ValueReder.
   * Liest mit append_from_file Config-Dateien ein.
   * Ersetzt bestehende Config-Dateien mit replace_config_file.
   * Eintrage der Form "+ FILENAME" und "++ FILENAME" fuehren zu rekursivem Auswerten von Konfigurationsdateien.
   * Unterschied zwischen "+" und "++": bei "+" werden die Blocknamen in der rekursiv eingelesenen Datei 
   * unabhaengig von den Blockmarkierungen in der uebergeordneten Datei gefuehrt, d.h. es wird mit [] 
   * gestartet und eine Ueberschrift in einer rekursiv eingelesenen Datei veraendert die Ueberschrift in der 
   * uebergeordneten nicht. Bei "++" werden die Ueberschriften fortgefuehrt, d.h. eine Ueberschrift in einer 
   * untergeordneten Datei wirkt sich auch auf die uebergeordneten aus und umgekehrt.
   * In allen Eintraegen wird das Kuerzel "$PATH" durch den Pfadnamen ersetzt, in dem 
   * sich die aktuell gelesene Konfigurationsdatei befindet.
   * In allen Eintraegen wird das Kuerzel "$HOME" durch den Inhalt der Environment-Variable $HOME ersetzt.
   * Blockmarkierungen [Ueberschrift] kann man abfragen mit get ("Ueberschrift::Zeile", ...).
   * Leere Blockmarkierung [] fuehrt zu keinen Prefix.
   * Verbosity:
   * 0 -> keine Ausgabe.
   * 1 -> fehlerhafte Zeilen melden.
   * 2 -> lesen von Konfig-Files melden (auf stderr).
   * 3 -> lesen von korrekten Zeilen melden.
   * 4 -> korrekte Zeilen mit Werten melden.
  */
  class ConfigReader {
  struct CmdLineShortcut {
    std::string short_key;
    std::string long_key;
    bool has_arg;
  };
  public:
    /** ConfigReader erzeugen, 
        arg1=verbosity
        arg2=Kommentarzeichen
        arg3=Trennzeichen */
    ConfigReader (unsigned int=0, char ='#', char ='=') throw ();
    ConfigReader (const ConfigReader&) throw (std::bad_alloc);
    /** Destruktor */
    ~ConfigReader () throw ();

    /** eine Definition fuer eine Kommandozeilenshortcut hinzufuegen
        Arg1: Kurzname
        Arg2: Langname
        Arg3: mit Argument? */
    void add_command_line_shortcut (const char*, const char*, bool);

    /** Informationen aus der Kommandozeile einfuegen, 
        Arg1 und Arg2 sind die Argumente argc und argv aus der main-Funktion.
        Argumente der Form --Key=Value werden direkt als Schluessel-Wert-Paare
        interpretiert. Argumente der Form -Key hingegeb werden mit den in
        add_command_line_shortcut() uebrgebenen Kurzschluesseln abgeglichen
        und der kurze Schluessel durch den langen Schluessel ersetzt. Ggf.
        wird der Rest des Arguments oder das naechste Argument als Wert
        fuer diesen Schluessel interpretiert, andernfalls wird der Wert auf 
        '1'/'true' gesetzt. Argumente ohne fuehrendes '-' die nicht als
        Argumente voranstehender -Key Optionen interpretiert werden koennen,
        werden den Schluesseln 'ConfigReader::unknown_argument_1', ... 
        zugeordnet. argv[0] wird als Programmname interpretiert und dem
        Schluessel 'ConfigReader::program_name' zugewiesen */
    void append_from_command_line (unsigned int, const char**);
    void append_from_command_line (unsigned int, char**);

    /** Information aus Datei einfuegen
        Arg1: Dateiname
        Arg2: sollen rekursiv andere Dateien gelesen werden (+ -Direktiven)?
        Ret: konnte aus allen Dateien gelesen werden? */
    bool append_from_file (const char*, bool=true) throw (std::bad_alloc);

    /** ueberschreibt eine Konfigurationsdatei. Die Quellatei wird in eine .bak-Datei umbenannt und
    eine neue Datei geschrieben. Zu den in der Quelldatei auftretenden Schluesseln werden die
    gespeicherten Werte ausgegeben. Nicht auftretende Schluessel werden ignoriert.
    Arg1: Name der Konfigurationsdatei
    Arg2: sollen rekursiv andere Dateien gelesen werden (+ -Direktiven)?
    Ret: konnten alle Dateien gelesen und geschrieben werden? */	
    bool replace_config_file (const char*, bool =true) throw (std::bad_alloc);
    /** wie zuvor, allerdings wird nur der Wert der Schluessel aus arg2 geaendert */
    bool replace_config_file (const char*, const std::vector<std::string>&, bool =true) throw (std::bad_alloc);

    /** Liste aller gelesenen Dateien */
    const std::vector<std::string>& list_of_sources () const throw ();

    // Informationen holen: arg1=key, arg2=ret-value, 
    // ret=sucess bzw. Groesse des gelesenen Array
    // ret=0, falls keine Zeile gefunden wurdem
    // ret<0, falls Konvertierung des Strings fehlgeschlagen
    int get (const char*, int&) const throw (std::bad_alloc);                  ///< int lesen
    int get (const char*, unsigned int&) const throw (std::bad_alloc);         ///< unsigned int lesen
    int get (const char*, long int&) const throw (std::bad_alloc);             ///< long int lesen
    int get (const char*, unsigned long int&) const throw (std::bad_alloc);    ///< unsigned long int lesen
    int get (const char*, double&) const throw (std::bad_alloc);               ///< double lesen
    int get (const char*, float&) const throw (std::bad_alloc);                ///< double lesen
    int get (const char*, char&) const throw (std::bad_alloc);                 ///< char lesen
    int get (const char*, bool&) const throw (std::bad_alloc);                 ///< bool lesen (1/0/true/false)
    int get (const char*, std::string&) const throw (std::bad_alloc);          ///< string (ein Wort) lesen
    int getline (const char*, std::string&) const throw (std::bad_alloc);          ///< string (ganze Zeile) lesen

    int get (const char*, std::vector<int>&) const throw (std::bad_alloc);
    int get (const char*, std::vector<unsigned int>&) const throw (std::bad_alloc);
    int get (const char*, std::vector<long int>&) const throw (std::bad_alloc);
    int get (const char*, std::vector<unsigned long int>&) const throw (std::bad_alloc);
    int get (const char*, std::vector<double>&) const throw (std::bad_alloc);
    int get (const char*, std::vector<float>&) const throw (std::bad_alloc);
    int get (const char*, std::vector<char>&) const throw (std::bad_alloc);
    int get (const char*, std::vector<bool>&) const throw (std::bad_alloc);
    int get (const char*, std::vector<std::string>&) const throw (std::bad_alloc);

    void set (const char*, int) throw (std::bad_alloc);                  ///< int schreiben
    void set (const char*, unsigned int) throw (std::bad_alloc);         ///< unsigned int schreiben
    void set (const char*, long int) throw (std::bad_alloc);             ///< long int schreiben
    void set (const char*, unsigned long int) throw (std::bad_alloc);    ///< unsigned long int schreiben
    void set (const char*, double) throw (std::bad_alloc);               ///< double schreiben
    void set (const char*, float) throw (std::bad_alloc);                ///< double schreiben
    void set (const char*, char) throw (std::bad_alloc);                 ///< char schreiben
    void set (const char*, bool) throw (std::bad_alloc);                 ///< bool schreiben (true/false)
    void set (const char*, const std::string&) throw (std::bad_alloc);   ///< string (ein Wort) schreiben
    void set (const char*, const char*) throw (std::bad_alloc);          ///< string (ein Wort) schreiben

    void set (const char*, const std::vector<int>&) throw (std::bad_alloc);
    void set (const char*, const std::vector<unsigned int>&) throw (std::bad_alloc);
    void set (const char*, const std::vector<long int>&) throw (std::bad_alloc);
    void set (const char*, const std::vector<unsigned long int>&) throw (std::bad_alloc);
    void set (const char*, const std::vector<double>&) throw (std::bad_alloc);
    void set (const char*, const std::vector<float>&) throw (std::bad_alloc);
    void set (const char*, const std::vector<char>&) throw (std::bad_alloc);
    void set (const char*, const std::vector<bool>&) throw (std::bad_alloc);
    void set (const char*, const std::vector<std::string>&) throw (std::bad_alloc);

    /** einen Eintrag in einen Stream schreiben; liefert true bei Erfolg, d.h. wenn Schluessel gefunden */
    bool write (std::ostream&, const char*) const throw ();

    /** Alle Schlussel-Wert-Paare in einen stream schreiben */
    void write (std::ostream&) const throw ();

    /** Alle Schlussel-Wert-Paare einer Section (arg2) in einen stream schreiben */
    void write_section (std::ostream&, const char*) const throw ();

  protected:
    struct KVPair;
    std::vector<KVPair> key_value_table;        // lexikographisch sortierte Liste mit Schluessel-Wert-Paaren
    const unsigned int verbosity;              // ermoeglicht verschiedene Kontrollausgaben, siehe oben
    const char comment_char;                   // Zeichen, um Kommentarzeilen einzuleiten
    const char assign_char;                    // Zeichen, um Schluessel-Wert-Zuweisungen zu markieren
    std::stack<std::string> directory_stack;   // beim Einlesen verschachtelter Aufrufe zum Zwischenspeichern des aktuellen Verzeichnisses
    std::vector<std::string> files;             // Liste aller eingelesener Dateien
    std::string home_path;                     // der Wert der Environment-Variable $HOME

    std::vector<CmdLineShortcut> shortcuts;    // Kommandozeilenshortcuts (mit einfachem '-')
    unsigned int unknown_parameter_counter;    // Zaehler, um unbekannte Kommandozeilenargumente zu zaehlen

    bool find_first (std::string& value, const std::string& key) const;  ///< finde erstes Wort im Valueteil zum Schluessel key, liefere true bei Erfolg
    bool find_all (std::deque<std::string>& values, const std::string& key) const;  ///< finde alle Worte im Valueteil zum Schluessel key, liefere true bei Erfolg

    /** vereinigt append_from_file und replace_config_file
      Arg1: Appendix fuer backups bei replace_config_file, NULL bei append_config_file
      Arg2: Name der Konfigurationsdatei
      Arg3: Schluessel, die ersetzt werden sollen. Leer=alle Schluessen ersetzen
      Arg4: sollen rekursiv andere Dateien gelesen werden (+ -Direktiven)?
      Arg5: der Praefix, der verwendet werden soll (Eingabe) bzw. der letzte Praefix, der verwendet wurde (Rueckgabe)
      Ret: konnten alle Dateien gelesen und geschrieben werden? */
    bool file_input (const char*, const char*, const std::vector<std::string>&, bool, std::string&) throw (std::bad_alloc);

    /** Schluessel(arg1)-Wert(arg2)-Paar einfuegen */
    void insert_pair (const std::string&, const std::string&) throw (std::bad_alloc);

  };

}

#endif
