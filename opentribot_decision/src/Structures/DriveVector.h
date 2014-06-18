
#ifndef tribots_drive_vector_h
#define tribots_drive_vector_h

#include "../Fundamental/Vec.h"
#include <iostream>
namespace Tribots {

  enum DriveVectorMode {ROBOTVELOCITY, WHEELVELOCITY, MOTORVOLTAGE};
	enum {NOKICK=0, HIGHKICK=1, LOWKICK=2};

  /** Struktur, um einen Fahr- und Kickbefehl darstellen und kommunizieren zu koennen; 
      alle Angaben in ROBOTERKOORDINATEN */
  struct DriveVector {
    /** Default-Konstruktor */
    DriveVector () throw ();
    /** Konstrukor
	Arg1: vtrans
	Arg2: vrot
	Arg3: Schusserlaubnis 
    Arg4: Schusslaenge in ms (später aus dem configfiles???*/
    
    DriveVector (Vec, double =0, unsigned int =NOKICK, unsigned int =150) throw();
    /** Konstrukor
	Arg1: vtrans.x (ROBOTVELOCITY), wheel vel 1 (WHEELVELOCITY), motor voltage 1 (MOTORVOLTAGE)
	Arg2: vtrans.y (ROBOTVELOCITY), wheel vel 2 (WHEELVELOCITY), motor voltage 2 (MOTORVOLTAGE)
	Arg3: vrot     (ROBOTVELOCITY), wheel vel 3 (WHEELVELOCITY), motor voltage 3 (MOTORVOLTAGE)
	Arg4: Kickpermission
	Arg5: mode (ROBOTVELOCITY, WHEELVELOCITY, MOTORVOLTAGE)
    Arg6: Schusslaenge ind ms**/
    DriveVector (double, double, double, unsigned int =NOKICK, DriveVectorMode =ROBOTVELOCITY, unsigned int =150);

    /** Copy-Konstruktor */
    DriveVector (const DriveVector&) throw ();
    /** Zuweisungsoperator */
    const DriveVector& operator= (const DriveVector&) throw ();
    
   
    Vec vtrans;    ///< translatorische Geschwindigkeit in m/s (im Roboterkoordinatensystem)
    double vrot;   ///< rotatorische Geschwindigkeit im Gegenuhrzeigersinn in rad/s (modus: ROBOTVELOCITY)
    unsigned int kick;     ///< Schussanforderung: NOKICK=0, HIGHKICK=1, LOWKICK=2
    DriveVectorMode mode;  ///< Modus, der beschreibt welche Daten ausgewertet werden und welche Bedeutung diese haben
    double vaux[3]; ///< Radgeschwindigkeiten oder Motorspannungen  (WHEELVELOCITY, MOTORVOLTAGE)
    unsigned int klength; //dauer der ausloesung des kickers in ms
  };


}
std::ostream& operator<< (std::ostream& os, const Tribots::DriveVector& v);

std::istream& operator>> (std::istream &in, Tribots::DriveVector &v);
#endif

