#ifndef _robotmask_h_
#define _robotmask_h_

#include <string>
#include <iostream>
#include "../../Structures/TribotsException.h"

namespace Tribots {

  /**
   * Stellt eine Maske bereit, in der ungültige Bildbereiche (Aufbauten) 
   * markiert sind. Hierzu kann ein PPM Bild (Binär, Typ 6) eingelesen 
   * werden. Gültige Bereiche müssen hier weiss (255,255,255), ungültige
   * schwarz markiert sein.
   *
   * Diese Klasse verwendet _nicht_ die Implementierung der Bildklasse,
   * da hier für die isValid - Methode eine möglichst niedrige Rechenzeit
   * erzielt werden sollte. Intern werden nicht die eingelesenen RGB-Werte
   * sondern nur true / false gespeichert. Diese Klasse ist aber zu den vom 
   * der PPMIO erzeugten Bildern kompatibel.
   */
  class RobotMask {
  public:
    /** lade eine neue Maske aus einer Datei; arg1=Dateiname */
    RobotMask (const char*) throw (TribotsException);
    /** Copy-Konstruktor */
    RobotMask (const RobotMask&) throw ();
    /** Zuweisungsoperator */
    const RobotMask& operator= (const RobotMask&) throw ();    
    /** erzeuge leere Maske der Breite arg1 und Hoehe arg2; arg3=default-Wert */
    RobotMask (unsigned int, unsigned int, bool = true);

    ~RobotMask();

    /** als PPM-File (P6) in einen Stream schreiben */
    void writeToStream (std::ostream&) const throw ();

    /** Position (x,y) anfragen */
    inline bool isValid(unsigned int x, unsigned int y) const throw ();
    /** Position (x,y) setzen */
    inline void set(unsigned int x, unsigned int y, bool val) const throw ();
    /** Breite der Maske anfragen */
    unsigned int getWidth () const throw ();
    /** Hoehe der Maske anfragen */
    unsigned int getHeight () const throw ();

    /** dialtiert die Maske, d.h. die verbotenen Bereiche werden um ein Pixel in alle Richtungen vergroessert, 9-Nachbarschaft */
    RobotMask* dilate () const throw (std::bad_alloc);
    /** eine ein Pixel breiten Rand mit ungueltigen Pixeln ergaenzen */
    void addFrame () throw ();

  protected:
    bool* mask;
    unsigned int width;
    unsigned int height;
  };

}

// inlines ////////////////////////////////////////////////////////////////////

bool Tribots::RobotMask::isValid(unsigned int x, unsigned int y) const throw ()
{
  return (x<width) && (y<height) && (mask[x+y*width]);  // lazy evaluation sei Dank
}

void Tribots::RobotMask::set(unsigned int x, unsigned int y, bool val) const throw () {
  if (x<width && y < height)
    mask[x+y*width]=val;
}

#endif
