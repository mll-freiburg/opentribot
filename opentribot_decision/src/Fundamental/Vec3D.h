
#ifndef Vec3D_h
#define Vec3D_h

#include "Angle.h"
#include "Vec.h"
#include <iostream>

namespace Tribots {

  /** 3-dimensional vector
   *
   *  This implementation of an 3-Dimensional vector only provides the most basic
   *  functionality - not all methods of the class Vec are available.
   *
   *  Most methods which involve mirroring or rotation only work on the x and y
   *  dimensions (adapted from Vec.h) and leave the z component untouched.
   */
  class Vec3D {

  public:
    double x,y,z;    ///< Wert in 3. Dimension

    /** Default-Konstruktor */
    Vec3D () throw () : x(0.), y(0.), z(0.) {;}
    /** Konstruktor zum direkten Setzen der Werte 3D*/
    Vec3D (double x1, double y1, double z1) throw () : x(x1), y(y1), z(z1) {;}
    /** Copy-Konstruktor 2D*/
    Vec3D (const Vec& v) throw () : x(v.x), y(v.y), z(0.0) {;}
    /** Copy-Konstruktor 3D*/
    Vec3D (const Vec3D& v) throw () : x(v.x), y(v.y), z(v.z) {;}
    /** Zuweisungsoperator 2D*/
    const Vec3D& operator= (const Vec& v) { x=v.x; y=v.y; z=0.0; return *this; }
    /** Zuweisungsoperator 3D*/
    const Vec3D& operator= (const Vec3D& v) { x=v.x; y=v.y; z=v.z; return *this; }
    
    /** Projiziert den 3-dimensionalen Vektor orthogonal auf die x-y-Ebene und gibt einen 
     *  Vec zurück. */
    Vec toVec() const throw() { return Vec(x,y); } 

    /** Vergleichsoperator */
    bool operator== (const Vec3D) const throw ();
    bool operator!= (const Vec3D) const throw ();
    
        
    /** Addition */
    Vec3D operator+ (const Vec3D) const throw ();
    const Vec3D& operator+= (const Vec3D) throw ();
    /** Subtraktion */
    Vec3D operator- (const Vec3D) const throw ();
    const Vec3D& operator-= (const Vec3D) throw ();
    /** Negation */
    Vec3D operator- () const throw ();
    /** Skalarmultiplikation */
    const Vec3D& operator*= (double) throw ();
    const Vec3D& operator/= (double) throw ();       // Division durch Null wird nicht abgefangen, ergibt nan
    /** Skalarprodukt */
    double operator* (const Vec3D) const throw ();

    /** Cross-Product (Kreuzprodukt) */
    Vec3D crossProduct(const Vec3D) const throw ();
    
    // Achtung : Rotiationen nur um x/y Achsen (nicht z) - identisch mit Vec Methoden!
    
    /** Rotation um Winkel arg1, identisch mit Methode rotate (.) */
    Vec3D operator* (const Angle) const throw ();
    /** Rotation von (*this) um Winkel arg, identisch mit Methode s_rotate (.) */
    const Vec3D& operator*= (const Angle) throw ();
    /** Rotation um Winkel -arg1 */
    Vec3D operator/ (const Angle) const throw ();
    /** Rotation von (*this) um Winkel -arg1 */
    const Vec3D& operator/= (const Angle) throw ();

    
    // Rotationen:
    Vec3D rotate (const Angle) const throw ();       ///< Rotation um beliebigen Winkel
    Vec3D rotate_twelvth () const throw ();          ///< Rotation um 30 Grad
    Vec3D rotate_eleven_twelvth () const throw ();   ///< Rotation um -30=330 Grad
    Vec3D rotate_eighth () const throw ();           ///< Rotation um 45 Grad
    Vec3D rotate_seven_eighth () const throw ();     ///< Rotation um -45=315 Grad
    Vec3D rotate_sixth () const throw ();            ///< Rotation um 60 Grad
    Vec3D rotate_five_sixth () const throw ();       ///< Rotation um -60=300 Grad
    Vec3D rotate_quarter () const throw ();          ///< Rotation um 90 Grad
    Vec3D rotate_three_quarters () const throw ();   ///< Rotation um -90 Grad
    Vec3D rotate_half () const throw ();             ///< Rotation um 180 Grad=Punktspiegelung am Ursprung

    Vec3D s_rotate (const Angle) throw ();           ///< Rotation von this

    
    // Spiegelungen:
    Vec3D mirror (const Vec3D) const throw ();       ///< Spiegelung an einer Achse mit gegebener Richtung wenn ||Arg1|| > 0, ansonsten Punktspiegelung 
    Vec3D mirror_x () const throw ();                ///< Spiegelung an x-Achse
    Vec3D mirror_y () const throw ();                ///< Spiegelung an y-Achse
    Vec3D mirror_z () const throw ();                ///< Spiegelung an z-Achse
    Vec3D mirror_eighth () const throw ();           ///< Spiegelung an 1. Winkelhalbierender
    Vec3D mirror_three_eighth () const throw ();     ///< Spiegelung an 2. Winkelhalbierender
               
    /** Quadrierte Laenge des Vektors */
    double squared_length () const throw ();
    /** Laenge des Vektors */
    double length () const throw ();
    /** Winkel des Vektors (0 fuer den Nullvektor) */
    Angle angle () const throw ();
    /** Normalisieren ((0,0) beim Nullvektor) */
    Vec3D normalize () const throw ();
    
    /** Winkel zwischen zwei Vektoren (nan bei Nullvektor) */
    Angle angle (const Vec3D) const throw ();
    
    /** liefert einen Vektor der Laenge 1 in gegebener Richtung */
    static Vec3D unit_vector (Angle) throw ();

    static const Vec3D unit_vector_x;           ///< Einheitsvektor in x-Richtung
    static const Vec3D unit_vector_y;           ///< Einheitsvektor in y-Richtung
    static const Vec3D unit_vector_z;           ///< Einheitsvektor in z-Richtung
    static const Vec3D zero_vector;             ///< Nullvektor
    
  };

  /** Skalarmultiplikation */
  Vec3D operator* (Vec3D, double) throw ();
  Vec3D operator* (double, Vec3D) throw ();
  Vec3D operator/ (Vec3D, double) throw ();    // Division durch 0 wird nicht abgefangen, ergibt nan

  /** Lineare Unabhaengigkeit */
  bool linearly_independent (const Vec3D, const Vec3D);

  
}    

std::ostream& operator<< (std::ostream& os, const Tribots::Vec3D& v);

#endif
  

