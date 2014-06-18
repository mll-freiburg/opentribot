
#ifndef Tribots_VisualPositionOptimiser_h_
#define Tribots_VisualPositionOptimiser_h_

#include "../../Structures/VisibleObject.h"
#include "FieldLUT.h"

namespace Tribots {

  /** Klasse ermoeglicht die Optimierung einer Roboterposition anhand der visuellen
      Information mit Hilfe eines iterativen Optimierungsansatzes */
  class VisualPositionOptimiser {
  private:
    const FieldLUT& the_field_lut;  /// Referenz auf FieldLUT, die verwendet werden soll
    double c;                         /// width parameter of error function 1-(c*c)/(c*c+x*x)
    double c2;                      /// c*c, for short
    double d2;                     /// Breitenparameter^2 fuer Abstandsgewichte
    std::vector<double> weights;   /// Gewichtsmatrix fuer jedes Liniensegment
    
  protected:
    /** berechne den Fehler und die Ableitung des Fehlers,
        Argumente: Fehler(return), Ableitung nach x(return), Ableitung nach y(return), Ableitung nach phi(return),
	x, y, phi(in rad), Liste mit gesehenen Linien, Anzahl max. zu beruecksichtigender Liniensegmente
	Koordinaten in Weltkoordinatensystem mit einheitlich: y-Achse weist auf blaues Tor
	Konvention: Gewichts-Array "weights" muss vorher gesetzt worden sein */
    void error (double&, double&, double&, double&, double, double, double, const VisibleObjectList&, unsigned int) const throw ();
  public:
    /** Kostruktor, uebergeben wird FieldLUT, Breite der Fehlerverteilung und Breite der Entfernunggewichtsfunktion */
    VisualPositionOptimiser (const FieldLUT&, double, double) throw ();
    /** Entfernungsgewichte berechnen; 
	arg1: Liniensegmente
	arg2: Anzahl max. zu beruecksichtigender Liniensegmente
	Return: Summe der Gewichte */
    double calculate_distance_weights (const VisibleObjectList&, unsigned int =10000) throw ();
    /** Minimiere den Fehler mit RProp-artigem Update 
	Ergebnisrueckgabe ueber Argumente
	arg1: Anfangs- und Endposition x
	arg2: Anfangs- und Endposition phi
	arg3: Liste weiser Linien. Hier duerfen NUR WEISE LINIEN enthalten sein!
	arg4: Anzahl Iterationen 
	arg5: Anzahl max. zu beruecksichtigender Liniensegmente
	Return: Fehler vor letzter Iteration des Optimierers 
	Konvention: Entfernungsgewichte muessen zuvor berechnet worden sein */
    double optimise (Vec&, Angle&, const VisibleObjectList&, unsigned int, unsigned int =10000) const throw ();
    /** Berechne Kruemmung des Fehlers (2. Ableitung) an einer Position, Annahme: Hessematrix besitzt Diagonalform 
	Ergebnisrueckgabe ueber Argumente
	arg1: d2err/(dx)^2 und d2err/(dy)^2
	arg2: d2err/(dphi)^2
	arg3: (x,y)
	arg4: phi
	arg5: Liste weiser Linien. Hier duerfen NUR WEISE LINIEN enthalten sein!
	arg6: Anzahl max. zu beruecksichtigender Liniensegmente
	Rueckgabewert: Fehler an der betreffenden Position
	Konvention: Entfernungsgewichte muessen zuvor berechnet worden sein */
    double analyse (Vec&, double&, Vec, Angle, const VisibleObjectList&, unsigned int =10000) const throw ();
  };

}

#endif
