
#ifndef _TribotsTools_DistanceCalibrationImageAnalysis_h_
#define _TribotsTools_DistanceCalibrationImageAnalysis_h_

#include "MarkerLog.h"
#include "../../Fundamental/Time.h"
#include "../../ImageProcessing/Formation/Image.h"
#include "../../ImageProcessing/PixelAnalysis/RobotMask.h"
#include <vector>

namespace TribotsTools {

  class AngleKalmanFilter {
  public:
    AngleKalmanFilter (double, double, double, double);  ///< Argumente: varalpha, vartheta, varobs, beta
    ~AngleKalmanFilter ();

    void setInitial (double, double);  ///< initiale Position und Geschwindigkeit
    void predict (double, double);  ///< Argumente: Sollgeschwindigkeit (in rad/s), Zeitintervall (in s)
    void update (double);  ///< Argumente: beobachtete Richtung

    double getAngle () const;
    double getVelocity () const;
    double getPredictedAngle () const;
    double getPredictedVelocity () const;

  private:
    double alpha;
    double theta;
    double p11;
    double p12;
    double p22;
    double p11hat;
    double p12hat;
    double p22hat;
    double alphahat;
    double thetahat;

    double varalpha;
    double vartheta;
    double varobs;
    double beta;
  };

  struct ScanArea {
    unsigned char color;
    double begin;
    double end;
  };

  class DistanceCalibrationImageAnalysis {
  public:
    enum { ColorIgnore=0, ColorRedPatch, ColorBluePatch };

    DistanceCalibrationImageAnalysis ();
    ~DistanceCalibrationImageAnalysis ();

    std::vector<MarkerLog> nextImage (Tribots::Image& img, bool showCenter, bool showDirection, bool showMask, bool showClasses, bool showTransitions);

    void setRotation (double velocity);
    void setCenter (int cx, int cy);
    void setMask (Tribots::RobotMask*);
    void setSearchDirection (Tribots::Angle);
    void setEdgeDetectionSensitivity (unsigned int);

    void getCenter (int& x, int& y) const;
  private:
    Tribots::RobotMask* robotMask;
    int centerX, centerY;
    AngleKalmanFilter kfilter;
    Tribots::Time latestUpdateTime;
    double thetasoll;
    bool timestampUpdate;
    unsigned int edgeDetectionSensitivity;
    double pmangle;

    std::vector<MarkerLog> findTransitions (const Tribots::Image&, double);  ///< Bild und Soll-Rotationsgeschwindigkeit
    void writeCenter (Tribots::Image&);
    void writeDirection (Tribots::Image&);
    void writeMask (Tribots::Image&);
    void writeColorClasses (Tribots::Image&);
    void writeTransitions (Tribots::Image&, const std::vector<MarkerLog>&);
    double checkDirection (const Tribots::Image&, Tribots::Angle);  ///< prueft, wie viele Pixel in der angegebenen Richtung nicht ausmaskiert sind. Rueckgabewert ist freier Bereich geteilt durch Bildhoehe
    std::vector<ScanArea> scan (const Tribots::Image&, Tribots::Angle, double =0);  ///< im Bild arg1 einen Scan vom Mittelpunkt in Richtung arg2 machen und solche Bereiche liefern, die eine Mindestlaenge von arg3 haben
    std::vector<double> scanEdge (const Tribots::Image& img, Tribots::Angle, double =15);  ///< im Bild arg1 einen Scan von Mittelpunkt in Richtung arg2 machen und nach Gradienten mit Betrag>arg3 suchen. zurueckgegeben wird Liste mit Abstaenden, wobei negative Abstaende Stellen mit negativem Gradienten bedeuten.
  };

}

#endif
