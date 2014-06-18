
#ifndef _Tribots_CameraOptics_h_
#define _Tribots_CameraOptics_h_

#include "../../Fundamental/geometry.h"
#include "../../Fundamental/geometry3D.h"
#include "../../ImageProcessing/PixelAnalysis/CoordinateMapping.h"
#include <stdexcept>

namespace Tribots {

  class CameraOptics {
    // Intrinsische Parameter:
    double sx;  ///< Aufloesung x
    double sy;  ///< Aufloesung y
    double u0;  ///< Hauptpunkt x
    double v0;  ///< Hauptpunkt y

    // Linsenverzerrung:
    double k1;  ///< radiale Verzerrung 2. Ordnung
    double k2;  ///< radiale Verzerrung 4. Ordnung
    double p1;  ///< tangentiale Verzerrung
    double p2;  ///< tangentiale Verzerrung

    // Extrinsische Parameter:
    Vec3D translation;  ///< Translation
    Vec3D rotationColumn1;  ///< Rotationsmatrix, 1. Spalte
    Vec3D rotationColumn2;  ///< Rotationsmatrix, 2. Spalte
    Vec3D rotationColumn3;  ///< Rotationsmatrix, 3. Spalte

    // Hilfsvariablen zur schnelleren Berechnung (werden von recalculateHelpers gesetzt)
    Vec3D RtT;  ///< Rotationsmatrix transponiert * Translationsvektor
    Vec3D RtAi1; ///< Rotationsmatrix transponiert * intrinsische Kameramatrix invertiert, 1. Spalte
    Vec3D RtAi2; ///< Rotationsmatrix transponiert * intrinsische Kameramatrix invertiert, 2. Spalte
    Vec3D RtAi3mRtT; ///< Rotationsmatrix transponiert * intrinsische Kameramatrix invertiert, 3. Spalte minus RtT

    /** Die Hilfsvariablen AT, AR1, AR2, AR3, RtT, RtAi1, RtAi2, RtAi3 berechnen */
    void recalculateHelpers () throw ();

  public:
    CameraOptics () throw ();

    /** schreibe die Parameter in einen Stream */
    void writeParametersToStream (std::ostream&) const throw ();
    /** lese Kameraparameter aus einem Stream und liefer true bei Erfolg */
    bool readParametersFromStream (std::istream&) throw ();

    /** Intrinsische Parameter setzen: Skalierung x, y, Hauptpunkt x, y */
    void setIntrinsicParameters (double sx1, double sy1, double u01, double v01) throw ();
    /** Hauptpunkt der Kamera verschieben um arg1 Pixel, sonst alles gleich lassen */
    void shiftPrinciplePoint (Vec) throw ();
    /** Extrinsische Parameter setzen: Translation, erste und zweite Spalte der Rotationsmatrix */
    void setExtrinsicParameters (Vec3D translation1, Vec3D rotation1, Vec3D rotation2) throw ();
    /** Roll/Pitch/Yaw-Winkel setzen */
    void setRollPitchYaw (Angle roll, Angle pitch, Angle yaw) throw ();
    /** linsenverzerrung festlegen: k1, k2, p1, p2 */
    void setDistortionParameters (double k11, double k21, double p11, double p21) throw ();

    /** berechne verzerrte Bildposition gegeben unverzerrte Bildposition p */
    Vec distort (Vec p) const throw ();

    /** berechne Abbildung des Weltpunktes p in das Bild ohne Beruecksichtigung der Verzerrung. Wirft eine exception, wenn der amgefrage Punkt hinter der Kamera liegt */
    Vec pinholeMap (Vec3D p) const throw (std::invalid_argument);
    /** berechne die Gerade, auf der alle Weltpunkte liegen, die auf p abgebildet werden */
    Line3D inversePinholeMap (Vec p) const throw ();
    /** berechnet die Richtung, auf der alle Weltpunkte liegen, die auf p abgebildet werden.
        azimuth liefert den Winkel projeziert auf die Grundflaeche,
        inclination liefert den vertikalen Winkel. 0 Grad bedeutet horizontale Richtung, positive Winkel Richtungen oberhalb des Horizonts, negative Winkel Richtungen unterhalb des Horizonts */
    void inversePinholeMap (Angle& azimuth, Angle& inclination, Vec p) const throw ();

    /** berechne Abbildung des Weltpunktes p in das Bild unter Beruecksichtigung der Verzerrung */
    Vec map (Vec3D p) const throw (std::invalid_argument);

    /** den Standort der Kamera berechnen */
    Vec3D cameraOrigin () const throw ();
    /** aus der Rotationsmatrix die Drehwinkel berechnen */
    void getRollPitchYaw (Angle& roll, Angle& pitch, Angle& yaw) const throw ();
    /** den Bereich der x-y-Ebene, der aufgrund der Kameraoptik nicht
        beobachtet werden kann (Punkte liegen hinter der Kamera).
        Liefert exception, wenn die Kameraache orthogonal zur x-y-Ebene
        steht */
    Halfplane nonObservableHalfplane () const throw (std::invalid_argument);
  };


  /** Abbildung, die ein Bild gegeben den Cameraparametern verzerrt */
  class DistortionMapping : public CoordinateMapping {
  public:
    /** Abbildung berechnen aus der Kameraoptik optics mit Bildbreite w und Bildhoehe h */
    DistortionMapping (const CameraOptics& optics, unsigned int w, unsigned int h) throw (std::bad_alloc);
    ~DistortionMapping () throw ();
    Vec map(const Vec&) const;
    unsigned int getWidth() const throw ();
    unsigned int getHeight() const throw ();

  private:
    unsigned int width;
    unsigned int height;
    Vec* mapping;  ///< width x height-Matrix, Zeileweise aufgebaut, die zu jedem Pixel den verzerrten Punkt enthaelt

    void rebuildMapping (const CameraOptics& optics) throw (std::bad_alloc);
  };


  /** Abbildung, die ein Bild gegeben den Cameraparametern entzerrt */
  class UndistortionMapping : public CoordinateMapping {
  public:
    /** Abbildung berechnen aus der Kameraoptik optics mit Bildbreite w und Bildhoehe h */
    UndistortionMapping (const CameraOptics& optics, unsigned int w, unsigned int h) throw (std::bad_alloc);
    ~UndistortionMapping () throw ();
    Vec map(const Vec&) const;
    unsigned int getWidth() const throw ();
    unsigned int getHeight() const throw ();

  private:
    int width;
    int height;
    Vec* mapping;  ///< width x height-Matrix, Zeileweise aufgebaut, die zu jedem Pixel den entzerrten Punkt enthaelt

    void rebuildMapping (const CameraOptics& optics) throw (std::bad_alloc);
  };

}

#endif
