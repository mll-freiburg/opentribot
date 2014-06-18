
#ifndef _TribotsTools_PolicyFactory_h_
#define _TribotsTools_PolicyFactory_h_

#include "Policy.h"
#include "../../../../Structures/TribotsException.h"
#include "../../../../Fundamental/ConfigReader.h"
#include <string>
#include <vector>
#include <map>


namespace TribotsTools {

  /** Abstrakte Klasse zum Erzeugen von Teamstrategien; fuer jede Strategie
      muss eine eigene Klasse abgeleitet werden, die die Klasse bei der PolicyFactory
      anmeldet. Als Beispiel siehe die PolicyDummy-Klasse */
  class PolicyBuilder {
  public:
    /** Destruktor */
    virtual ~PolicyBuilder () throw () {;}
    /** erzeuge eine neue Instanz des jeweiligen Strategie; werfe eine Exception, wenn
	Objekt nicht ordnungsgemaess erzeugt werden kann; 
	arg1: Typbezeichnung, arg2: Config-Parameter */
    virtual Policy* get_policy (const std::string&, const Tribots::ConfigReader&) throw (Tribots::TribotsException,std::bad_alloc) =0;
  };



  /** PolicyFactory verwaltet die verschiedenen Typen von Strategien
      Jede Strategie muss sich zunaechst bei der Factory anmelden */
  class PolicyFactory {
  private:
    std::map<std::string, PolicyBuilder*> list_of_plugins; ///< die Liste bekannter Strategien
    static PolicyFactory* the_only_factory;                ///< Zeiger auf die einzige Factory (singleton)
    PolicyFactory () throw ();                             ///< private Konstruktor, wegen singleton-Eigenschaft
    ~PolicyFactory() throw ();                             ///< Destruktor auch privat, Objekt soll nicht geloescht werden

  public:
    /** statt Konstruktor: statische Aufruffunktion */
    static PolicyFactory* get_policy_factory () throw (std::bad_alloc);

    /** Anmeldefunktion, arg1: Typbezeichner, arg2: Zeiger auf Builderobjekt */
    void sign_up (const std::string, PolicyBuilder*) throw (std::bad_alloc);
    /** Erzeugungsfunktion, arg1: Typbezeichner, arg2: Config-Parameter, return: neues Objekt; 
	bei Problemen werden Ausnahmen geworfen */
    Policy* get_policy (const std::string, const Tribots::ConfigReader&) throw (Tribots::TribotsException,std::bad_alloc,std::invalid_argument);
    /** schreibe in arg1 die Liste aller verfuegbaren Spielertypen */
    void policy_list (std::vector<std::string>&) const throw (std::bad_alloc);
  };

}

#endif
