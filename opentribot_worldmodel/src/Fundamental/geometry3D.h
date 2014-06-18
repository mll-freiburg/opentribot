
#ifndef _Tribots_geometry_3D_h_
#define _Tribots_geometry_3D_h_

#include "Vec.h"
#include "geometry.h"
#include "Vec3D.h"
#include <stdexcept>
#include <vector>


namespace Tribots {

  class Line3D;
  class LineSegment3D;
  
    
  /** Schnittpunkt zweier Geraden, die sich nicht schneiden */
  Vec3D intersectSkewLines(Vec3D L1Start, Vec3D L1Rel, Vec3D L2Start, Vec3D L2Rel);
  Vec3D intersectSkewLines(Line3D& l1, Line3D& l2);
  
  /** Schnittpunkt zweier Geraden; bei parallelen Geraden wird Ausnahme geworfen */
  Vec3D intersect (const Line3D&, const Line3D&) throw (std::invalid_argument);
  /** Schnittpunkte zwischen Geraden/Geradenstuecken */
  std::vector<Vec3D> intersect (const LineSegment3D&, const Line3D&) throw (std::bad_alloc);
  std::vector<Vec3D> intersect (const Line3D&, const LineSegment3D&) throw (std::bad_alloc);
  std::vector<Vec3D> intersect (const LineSegment3D&, const LineSegment3D&) throw (std::bad_alloc);

  /** Klasse Line modelliert eine Gerade im 3-dimensionalen */
  class Line3D {
  protected:
    friend Vec3D intersect (const Line3D&, const Line3D&) throw (std::invalid_argument);
    friend Vec3D perpendicular_point (const Vec3D&, const Line3D&) throw ();

  public:
    Vec3D p1;   // p1, p2: 2 Punkte auf der Linie (nicht identisch!)
    Vec3D p2;

    /** Konstruktor, uebergeben werden zwei Punkte der Linie;
	wirft eine invalid_argument Ausnahme, wenn beide Punkte identisch sind */
    Line3D (const Vec3D, const Vec3D) throw (std::invalid_argument);
    /** Line-Konstruktor */
    Line3D (const Line&) throw ();
    /** Copy-Konstruktor */
    Line3D (const Line3D&) throw ();
    /** Standard-Konstruktor, erzeugt leere Linie (von 0,0 bis 0,0) */
    Line3D () throw();
    /** Zuweisungsoperator */
    const Line3D& operator= (const Line3D&) throw ();

    /** calculates the intersection of this line with
     *  the horizontal plane of height z.
     *  \param z height of the 3D plane in world coordinates
     *  \returns Vec3D of the intersecting point (x,y,z)
     */
    Vec3D intersectZPlane(const double z) const;

    /** calculates the intersection of this line with
     *  the vertical plane at distance y (axis-symmetrical)
     * \param y distance of the 3D plane in world coordinates
     * \returns Vec3D of the intersecting point (x,y,z)
     */
    Vec3D intersectYPlane(const double y) const;

    /** liefert den Abstand eines Punktes zur Geraden */
    double distance (const Vec3D) throw ();

    /** Lotpunkt berechnen */
    Vec3D perpendicular_point (const Vec3D&) throw ();

    /** Rotation um Ursprung (um Z-Achse)*/
    Line3D& s_rotate   (const Angle&) throw();
    /** Translation */
    Line3D& s_translate(const Vec3D&) throw();

    /** Rotation um Ursprung (um z-Achse)*/
    Line3D rotate (const Angle) const throw ();
    /** Translation */
    Line3D translate (const Vec3D) const throw ();
  };
    
  /** Klasse LineSegment modelliert ein Linienstueck mit Anfangs- und Endpunkt */
  //TODO: Not implemented yet
  class LineSegment3D {
    friend std::vector<Vec3D> intersect (const LineSegment3D&, const Line3D&) throw (std::bad_alloc);
    friend std::vector<Vec3D> intersect (const LineSegment3D&, const LineSegment3D&) throw (std::bad_alloc);

  public:
    Vec3D p1;   // Anfangspunkt
    Vec3D p2;   // Endpunkt

    /** Konstruktor, uebergeben werden Anfangs- und Endpunkt;
	wirft eine invalid_argument Ausnahme, wenn beide Punkte identisch sind */
    LineSegment3D (const Vec3D, const Vec3D) throw (std::invalid_argument);
    /** 2D-LineSegment Konstruktor */
    LineSegment3D (const LineSegment&) throw ();
    /** Copy-Konstruktor */
    LineSegment3D (const LineSegment3D&) throw ();
    /** Zuweisungsoperator */
    const LineSegment3D& operator= (const LineSegment3D&) throw ();

    /** liefert den Abstand eines Punktes zum Linienstueck */
    double distance (const Vec3D) throw ();

    /** liefert den Anfangspunkt des Linienstuecks */
    const Vec3D& getStart() const throw();
    
    /** liefert den Endpunkt des Linienstuecks */
    const Vec3D& getEnd() const throw();

    /** Rotation um Ursprung (z-Achse)*/
    LineSegment3D& s_rotate   (const Angle&) throw();
    /** Translation */
    LineSegment3D& s_translate(const Vec3D&) throw();
    /** Rotation um Ursprung (z-Achse)*/
    LineSegment3D rotate (const Angle) const throw ();
    /** Translation */
    LineSegment3D translate (const Vec3D) const throw ();
  };    

}

std::ostream& operator<< (std::ostream& os, const Tribots::Line3D& v);

#endif





