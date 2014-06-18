
#ifndef _Tribots_FieldGeometry_h_
#define _Tribots_FieldGeometry_h_

#include "../Fundamental/ConfigReader.h"
#include "../Structures/TribotsException.h"
#include "../Fundamental/geometry.h"
#include <vector>
#include <string>


namespace Tribots {

  /** Struktur zur Repraesentation der Spielfeldgeometrie
      Alle Laengenangaben in mm */
  struct FieldGeometry {
    enum LinesetType {
      linesetNormal=0,  ///< normales Spielfeld gemaess Regelwerk
      linesetTriangle,  ///< Spielfeld mit Tribots-T-Dreieck statt Mittelkreis
      linesetTriangleInverted,  ///< Spielfeld mit Tribots-T-Dreieck statt Mittelkreis in umgekehrter Orientierung
      linesetWithoutCenter,  ///< Spielfeld ganz ohne Mittelkreis und Mittellinie
      linesetCrosswise  ///< Virtuelles Spielfeld, zu dem das echte Spielfeld einen Ausschnitt darstellt
    };
    /** Standard-Konstruktor */
    FieldGeometry () throw ();
    /** Konstruktor, der die Parameter aus einem ConfigReader liest */
    FieldGeometry (const ConfigReader&) throw (InvalidConfigurationException);
    /** Copy-Konstruktor erzeugt tiefe Kopie */
    FieldGeometry (const FieldGeometry&) throw ();
    /** Zuweisungsoperator erzeugt tiefe Kopie */
    const FieldGeometry& operator= (const FieldGeometry&) throw ();
    ~FieldGeometry () throw ();

    /** FieldGeometry in einen String schreiben */
    std::string serialize () const throw (std::bad_alloc);
    /** FieldGeometry aus einem String lesen. Liefert false, wenn nicht erfolgreich */
    bool deserialize (const std::string&) throw ();

    double field_length;           ///< Spielfeld-Laenge
    double field_width;            ///< Spielfeld-Breite

    double side_band_width;        ///< Breite des Bereichs neben den Seitenlinien
    double goal_band_width;        ///< Breite des Bereichs neben den Torauslinien

    double goal_area_length;       ///< Laenge (Tiefe) des Torraums
    double goal_area_width;        ///< Breite des Torraums
    double penalty_area_length;    ///< Laenge (Tiefe) des Strafraums
    double penalty_area_width;     ///< Breite des Strafraums
    double center_circle_radius;   ///< Radius der Kreisboegen an den Spielfeld-Ecken
    double corner_arc_radius;      ///< Radius des Mittelkreises
    double penalty_marker_distance;///< Abstand des Penaltymarkers vom Tor

    double line_thickness;         ///< Breite der weissen Linien im Innern des Feldes
    double border_line_thickness;  ///< Breite der weissen Linien zur Spielfeldbegrenzung

    double goal_width;             ///< Breite des Tores
    double goal_length;            ///< Laenge (Tiefe) des Tores
    double goal_height;            ///< Hoehe des Tores

    double pole_height;            ///< Hoehe der Eckpfosten; existieren keine Poles, pole_height=0 oder pole_diameter=0
    double pole_diameter;          ///< Durchmesser der Eckpfosten
    double pole_position_offset_x; ///< Position der Eckpfosten: Abstand zur Seitenlinie
    double pole_position_offset_y; ///< Position der Eckpfosten: Abstand zur Torauslinie

    double ball_diameter;          ///< Durchmesser des Balles

    LinesetType lineset_type;     ///< Anordnung der Feldlinien
    FieldGeometry* clipping_geometry;  ///< die Originalfeldgeometrie der Markierungen bei Feldausschnitten
    Vec clipping_offset;  ///< der Offset bei Feldausschnitten
    Angle clipping_angle;   ///< die Rotation bei Feldausschnitten

    // Moeglichkeit, um weitere Parameter einzufuegen


    /** Vergleich zweier Feldgeometrien; Abweichungen bis zu 10mm werden toleriert */
    bool operator== (const FieldGeometry&) const throw ();
    bool operator!= (const FieldGeometry&) const throw ();

    /** aus den Linien eine Menge von Linien und Boegen berechnen, die die Linien beschreiben, Rueckgabe in arg1 und arg2;
      Koordinatensystem: blaues Tor ist stets Richtung positiver y-Achse. Arg3 kontrolliert, ob bei Ausschnitten die tatsaechlich
      markierten Linien geliefert werden (true) oder die Linien des virtuellen Feldes (false) */
    void make_lineset (std::vector<LineSegment>&, std::vector<Arc>&, bool =true) const throw (std::bad_alloc);
  };

}

#endif
