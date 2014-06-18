
#ifndef _Tribots_geometry_h_
#define _Tribots_geometry_h_

#include "Vec.h"
#include <stdexcept>
#include <vector>
#include <iostream>
#include "Frame2D.h"


namespace Tribots {

  class Area;
  class Curve;
  class CurveAndArea;
  class Line;
  class LineSegment;
  class Circle;
  class Arc;
  class Triangle;
  class XYRectangle;
  class Quadrangle;
  class Halfplane;
  class UnionArea;

  /* Objekte mit Frame2d multiplizieren (Bewegung) */
  Line operator* (const Frame2d&, const Line&) throw ();
  LineSegment operator* (const Frame2d&, const LineSegment&) throw ();
  Arc operator* (const Frame2d&, const Arc&) throw ();
  Circle operator* (const Frame2d&, const Circle&) throw ();
  Triangle operator* (const Frame2d&, const Triangle&) throw ();
  Quadrangle operator* (const Frame2d&, const Quadrangle&) throw ();
  Halfplane operator* (const Frame2d&, const Halfplane&) throw ();

  /** Schnittpunkt zweier Geraden; bei parallelen Geraden wird Ausnahme geworfen */
  Vec intersect (const Line&, const Line&) throw (std::invalid_argument);
  /** Schnittpunkte zwischen Geraden/Geradenstuecken */
  std::vector<Vec> intersect (const LineSegment&, const Line&) throw (std::bad_alloc);
  std::vector<Vec> intersect (const Line&, const LineSegment&) throw (std::bad_alloc);
  std::vector<Vec> intersect (const LineSegment&, const LineSegment&) throw (std::bad_alloc);
  /** Schnittpunkte zwischen Gerade/Geradenstueck und Kreislinie */
  std::vector<Vec> intersect (const Line&, const Circle&) throw (std::bad_alloc);
  inline std::vector<Vec> intersect (const Circle& c, const Line& l) throw (std::bad_alloc) { return intersect (l,c); }
  std::vector<Vec> intersect (const LineSegment&, const Circle&) throw (std::bad_alloc);
  inline std::vector<Vec> intersect (const Circle& c, const LineSegment& l) throw (std::bad_alloc) { return intersect (l,c); }
  /** Schnittpunkte zweier Kreise; bei konzentrischen Kreisen wird Ausnahme geworfen */
  std::vector<Vec> intersect (const Circle&, const Circle&) throw (std::bad_alloc);
  /** Schnittpunkte zwischen Gerade und Kreisbogen */
  std::vector<Vec> intersect (const Line&, const Arc&) throw (std::bad_alloc);
  std::vector<Vec> intersect (const Arc&, const Line&) throw (std::bad_alloc);
  std::vector<Vec> intersect (const LineSegment&, const Arc&) throw (std::bad_alloc);
  std::vector<Vec> intersect (const Arc&, const LineSegment&) throw (std::bad_alloc);
  /** Schnittpunkte zwischen Viereck mit Krei, Linie, Liniensegment */
  std::vector<Vec> intersect (const Circle&, const Quadrangle&) throw (std::bad_alloc);
  inline std::vector<Vec> intersect (const Quadrangle& q, const Circle& c) throw (std::bad_alloc) { return intersect (c,q); }
  std::vector<Vec> intersect (const LineSegment&, const Quadrangle&) throw (std::bad_alloc);
  inline std::vector<Vec> intersect (const Quadrangle& q, const LineSegment& c) throw (std::bad_alloc) { return intersect (c,q); }
  std::vector<Vec> intersect (const Line&, const Quadrangle&) throw (std::bad_alloc);
  inline std::vector<Vec> intersect (const Quadrangle& q, const Line& c) throw (std::bad_alloc) { return intersect (c,q); }
  /** Tangentiale Punkte berechnen; wirft Ausnahme, falls Punkt innerhalb des Kreises */
  std::vector<Vec> tangent_point (const Circle&, const Vec&) throw (std::bad_alloc, std::invalid_argument);

  /** Strahlensatzanwendung "InterceptTheoremCorrection"
   * (um Projektion der gemessenen Ballposition zu berechnen:
   *  point == Abstand Roboter-Ball (auf dem Boden), b == Ballhoehe, h == Kamerahoehe) */
  Vec intercept_theorem(const Vec& point, double b, double h);

  /** ein Bereich der zweidimensionalen Ebene */
  class Area {
  public:
    virtual ~Area() throw () {;}
    /** pruefen, ob Argument in der Menge enthalten ist */
    virtual bool is_inside (Vec) const throw () =0;
    virtual void draw(std::ostream&) const throw () =0;
    /** erzeugt eine Kopie von der Area, bzw. von der Instanz der Unterklasse */
    virtual Area* clone() const =0;
  };

  /** eine kurvenförmige Struktur */
  class Curve {
  public:
    virtual ~Curve() throw () {;}
    /** naechstliegender Punkt auf der Kurve bestimmen */
    virtual Vec closest_point (const Vec) const throw () =0;
    /** kleinster Abstand von einem Punkt zu der Kurve */
    virtual double distance (const Vec p) const throw () { return (p-closest_point (p)).length(); }
    virtual void draw(std::ostream&) const throw () =0;
  };

  /** eine Struktur, die sowohl als Kurve als auch als Flaeche interpretiert werden kann */
  class CurveAndArea : public Curve, public Area {
  };
  
  /** Klasse Line modelliert eine Gerade im 2-dimensionalen */
  class Line : public Curve {
  protected:
    friend Vec intersect (const Line&, const Line&) throw (std::invalid_argument);
    friend std::vector<Vec> intersect (const Line&, const Circle&) throw (std::bad_alloc);
    friend std::vector<Vec> intersect (const Line&, const Arc&) throw (std::bad_alloc);
    friend Vec perpendicular_point (const Vec&, const Line&) throw ();
    friend Line operator* (const Frame2d&, const Line&) throw ();

  public:
    Vec p1;   // p1, p2: 2 Punkte auf der Linie (nicht identisch!)
    Vec p2;
    Line () throw ();
    /** Konstruktor, uebergeben werden zwei Punkte der Linie;
        wirft eine invalid_argument Ausnahme, wenn beide Punkte identisch sind */
    Line (const Vec, const Vec) throw (std::invalid_argument);
    /** Copy-Konstruktor */
    Line (const Line&) throw ();
    /** Zuweisungsoperator */
    const Line& operator= (const Line&) throw ();
    /** Destruktor */
    ~Line () throw () {;}

    /** liefert den Abstand eines Punktes zur Geraden */
    double distance (const Vec) const throw ();
    /** liefert die Seite, auf der Punkt bezueglich Geraden liegt
        Orientierung der Geraden von p1 nach p2;
        liefert -1 falls Punkt links liegt, 0 falls Punkt auf Geraden liegt, +1 falls Punkt rechts der Geraden liegt */
    int side (const Vec) throw ();
    /** Lotpunkt berechnen */
    Vec perpendicular_point (const Vec) const throw ();
    Vec closest_point (const Vec) const throw ();

    /** Rotation um Ursprung */
    Line& s_rotate   (const Angle&) throw();
    /** Translation */
    Line& s_translate(const Vec&) throw();

    /** Rotation um Ursprung */
    Line rotate (const Angle) const throw ();
    /** Translation */
    Line translate (const Vec) const throw ();
  
    void draw(std::ostream&) const throw ();
  };
    

  /** Klasse LineSegment modelliert ein Linienstueck mit Anfangs- und Endpunkt */
  class LineSegment : public Curve {
    friend std::vector<Vec> intersect (const LineSegment&, const Arc&) throw (std::bad_alloc);
    friend std::vector<Vec> intersect (const LineSegment&, const Line&) throw (std::bad_alloc);
    friend std::vector<Vec> intersect (const LineSegment&, const LineSegment&) throw (std::bad_alloc);
    friend LineSegment operator* (const Frame2d&, const LineSegment&) throw ();

  public:
    Vec p1;   // Anfangspunkt
    Vec p2;   // Endpunkt

    LineSegment () throw ();
    /** Konstruktor, uebergeben werden Anfangs- und Endpunkt;
        wirft eine invalid_argument Ausnahme, wenn beide Punkte identisch sind */
    LineSegment (const Vec, const Vec) throw (std::invalid_argument);
    /** Copy-Konstruktor */
    LineSegment (const LineSegment&) throw ();
    /** Zuweisungsoperator */
    const LineSegment& operator= (const LineSegment&) throw ();
    /** Destruktor */
    ~LineSegment () throw () {;}

    /** den naechstgelegenen Punkt bestimmen */
    Vec closest_point (const Vec) const throw ();

    /** liefert den Anfangspunkt des Linienstuecks */
    const Vec& getStart() const throw();
    
    /** liefert den Endpunkt des Linienstuecks */
    const Vec& getEnd() const throw();

    /** Rotation um Ursprung */
    LineSegment& s_rotate   (const Angle&) throw();
    /** Translation */
    LineSegment& s_translate(const Vec&) throw();
    /** Rotation um Ursprung */
    LineSegment rotate (const Angle) const throw ();
    /** Translation */
    LineSegment translate (const Vec) const throw ();
  
    void draw(std::ostream&) const throw ();
  };


  /** Klasse modelliert einen Kreisbogen */
  class Arc : public Curve {
  private:
    friend std::vector<Vec> intersect (const Line&, const Arc&) throw (std::bad_alloc);
    friend std::vector<Vec> intersect (const LineSegment&, const Arc&) throw (std::bad_alloc);
    friend Arc operator* (const Frame2d&, const Arc&) throw ();

  public:
    Vec center;   ///< Mittelpunkt des Bogens
    double radius;  ///< Radius
    Angle start;  ///< Anfangswinkel im mathematischen Sinn
    Angle end;  ///< Endwinkel im mathermatischen Sinn
  
    /** Konstruktor */
    Arc () throw ();
    Arc (Vec, double, Angle, Angle) throw ();
    /** Copy-Konstruktor */
    Arc (const Arc&) throw ();
    /** Zuweisungsoperator */
    const Arc& operator= (const Arc&) throw ();
    /** Destruktor */
    ~Arc () throw () {;}

    /** Rotation um Ursprung */
    Arc& s_rotate   (const Angle&) throw();
    /** Translation */
    Arc& s_translate(const Vec&) throw();

    /** Rotation um Ursprung */
    Arc rotate (const Angle) const throw ();
    /** Translation */
    Arc translate (const Vec) const throw ();
  
    /** den naechstgelegenen Punkt bestimmen */
    Vec closest_point (const Vec) const throw ();
  
    void draw(std::ostream&) const throw ();
  };


  /** Klasse Circle modelliert einen Kreis im 2-dimensionalen */
  class Circle : public CurveAndArea {
  private:
    friend std::vector<Vec> intersect (const Line&, const Circle&) throw (std::bad_alloc);
    friend std::vector<Vec> intersect (const Circle&, const Circle&) throw (std::bad_alloc);
    friend std::vector<Vec> tangent_point (const Circle&, const Vec&) throw (std::bad_alloc, std::invalid_argument);
    friend Circle operator* (const Frame2d&, const Circle&) throw ();

  public:
    Vec center;     ///< Mittelpunkt
    double radius;  ///< Radius
  
    /** Konstruktor, uebergeben werden Mittelpunkt und Radius */
    Circle () throw ();
    Circle (const Vec, double) throw ();
    /** Konstruktor, uebergeben werden drei Punkte des Kreises; bei kollinearen Punkten wird invalid_argument geworfen */
    Circle (const Vec, const Vec, const Vec) throw (std::invalid_argument);
    /** Copy-Konstruktor */
    Circle (const Circle&) throw ();
    /** Zuweisungsoperator */
    const Circle& operator= (const Circle&) throw ();
    
    Area* clone() const {  return new Circle(*this); }

    /** liefere Mittelpunkt */
    const Vec& get_center () const throw ();
    /** liefere Radius */
    double get_radius () const throw ();

    /** prueft, ob ein Punkt innerhalb des Kreises liegt (einschliesslich Rand) */
    bool is_inside (const Vec) const throw ();  
    /** liefert den Abstand eines Punktes zur Kreislinie, fuer innere Punkte -> negative Werte */
    double distance (const Vec) const throw ();
    /** liefert den naechstliegenden Punkt auf dem Kreis */
    Vec closest_point (const Vec) const throw ();
    void draw(std::ostream&) const throw ();
  };


  /** Klasse Triangle modelliert ein Dreieck */
  class Triangle : public Area {
  public:
    Triangle () throw ();
    Triangle (Vec,Vec,Vec) throw ();
    Triangle (const Triangle&) throw ();
    Area* clone() const { return new Triangle(*this); }
    bool is_inside (Vec) const throw ();
    double area () const throw ();  ///< Flaecheninhalt
    void draw(std::ostream&) const throw ();
    Vec p1, p2, p3;
    
    friend Triangle operator* (const Frame2d&, const Triangle&) throw ();
  };


  /** Klasse XYRectangle fuer achsenparalleles Rechteck */
  class XYRectangle : public Area {
  public:
    XYRectangle () throw ();
    XYRectangle (const XYRectangle&) throw ();
    /** Konstruktor, Argumente: 2 diagonale Eckpunkte */
    XYRectangle (Vec,Vec) throw ();
    Area* clone() const { return new XYRectangle(*this); }
    /** pruefen, ob ein Punkt innerhalb liegt (einschliesslich Rand) */
    bool is_inside (const Vec) const throw ();
    void draw(std::ostream&) const throw ();
    Vec p1;
    Vec p2;
  };

  
  /** Klasse Quadrangle fuer ein allgemeines Viereck */
  class Quadrangle : public CurveAndArea {
  public:
    Quadrangle () throw ();
    Quadrangle (const Quadrangle&) throw ();
    /** Konstruktor, Funktion wie bei XYRectangle */
    Quadrangle (Vec, Vec) throw ();
    /** Konstruktor, ubergeben werden zwei Punkte sowie die Breite des Korridors */
    Quadrangle (const Vec&, const Vec&, double) throw ();    
    /** Konstruktor, ubergeben werden zwei Punkte sowie die Anfangs und Endbreite des Korridors */
    Quadrangle (const Vec&, const Vec&, double,double) throw ();    
    /** Konstruktor, ubergeben werden die Eckpunkte umlaufend um das Viereck */
    Quadrangle (Vec, Vec, Vec, Vec) throw ();
    Area* clone() const { return new Quadrangle(*this); }
    bool is_inside (const Vec) const throw ();
    Vec closest_point (const Vec) const throw ();
    void draw(std::ostream&) const throw ();
    Vec p1, p2, p3, p4;

    friend Quadrangle operator* (const Frame2d&, const Quadrangle&) throw ();
  }; 
 

  /** Klasse Halfplane fuer eine Halbebene */
  class Halfplane : public Area {
  public:
    Halfplane () throw ();
    Halfplane (const Halfplane&) throw ();
    /** Konstruktor mit einem Punkt auf der Grenze (Arg1) und einem Normalenvektor (Arg2), der in das innere weist */ 
    Halfplane (Vec, Vec) throw ();
    /** Konstruktor mit einem Punkt auf der Grenze (Arg1) und einem Winkel (Arg2);
        die innersen Punkte der Halbebene sind diejenigen von denen man von Arg1 in einem Winkel
        zwischen arg2 und arg2+180Grad */
    Halfplane (Vec, Angle) throw ();
    Area* clone() const { return new Halfplane(*this); }
    bool is_inside (Vec) const throw ();
    void draw(std::ostream&) const throw ();
  private:
    Vec p1;
    Vec norm;

    friend Halfplane operator* (const Frame2d&, const Halfplane&) throw ();
  };

  /** Vereinigung mehrer Flaechen */
  class UnionArea : public Area {
  public:
    UnionArea () throw ();
    virtual ~UnionArea() throw ();
    virtual void add (const Area&) throw ();  ///< eine Flaeche hinzufuegen
    virtual bool is_inside (Vec) const throw ();
    virtual void draw(std::ostream&) const throw ();  ///< zeichnet alle Flaechen einzeln
    virtual Area* clone() const;
  private:
    std::vector<Area*> elements;
  };

}

/** Ausgabe-Schiebeoperator fuer Curve */
std::ostream& operator<< (std::ostream&, const Tribots::Curve&);
/** Ausgabe-Schiebeoperator fuer Area */
std::ostream& operator<< (std::ostream&, const Tribots::Area&);
/** Ausgabe-Schiebeoperator fuer CurveAndArea */
std::ostream& operator<< (std::ostream&, const Tribots::CurveAndArea&);

#endif

