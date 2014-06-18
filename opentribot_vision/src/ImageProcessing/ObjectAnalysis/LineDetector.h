#ifndef _linedetector_h_
#define _linedetector_h_

#include "LineScanning.h"
#include "FieldMapper.h"
#include "../Formation/Image.h"
#include "../PixelAnalysis/CoordinateMapping.h"
#include "../../Structures/VisibleObject.h"

namespace Tribots {

  class LineDetector {
  public:
    /**
     * Erzeugt und initialisiert einen Liniendetektoren mit der übergebenen
     * Koordinatentransformation (Bild->Welt) und den übergebenen Werten für
     * die minimale und maximale Breite von Linien.
     *
     * \param mapping Koordinatentransformation von Bild zu egozentrischen
     *                Weltkoordinaten
     * \param minLineWidth Übergänge, die dichter beieinander liegen, werden
     *                     aussortiert (Wert im Weltkoordinatensystem)
     * \param maxLineWidth Übergänge, die weiter auseinander liegen, werden
     *                     aussortiert (Wert im Weltkoordinatensystem)
     */
    LineDetector(const ImageWorldMapping* mapping, 
    		 FieldMapper * fieldmapper,
		 double minLineWidth,
		 double maxLineWidth,
		 bool use2Steps = false,
		 bool checkForGreen = false,
		 bool cutOutsideField = false,
		 bool useMaximalDistance = false,
		 double maximalDistance = 3000.,
		 bool useHalfField = false,
		 bool useFieldMapper =false);

    /**
     * Findet Linien in den übergebenen Resultaten des Linienscanners und
     * übergibt die gefundenen Punkte als VisibleObject an das Weltmodell.
     * Eine Linie besteht hier immer aus zwei Übergängen (zur und von der
     * Line). Für jede Paarung wird die Methode checkLinePlausibility 
     * aufgerufen, die verschiedene Plausibilitätstest durchführt und so
     * unwahrscheinliche Übergänge herausfiltert.
     *
     * \attention Ändert über einen Seiteneffekt das Weltmodell.
     *
     * \param scanResults ScanResult Struktur der Farbklasse COLOR_LINE
     * \param vol Wird hier eine VisibleObjectList übergeben, werden die
     *            Objekte nicht nur ins Weltmodell sondern auch an diese Liste 
     *            angehängt.
     * \param writeWM gibt an, ob die gefundenen Objekte ins Weltmodell
     *                geschrieben werden sollen
     */
    void searchLines(const ScanResult* scanResult, Time time,
		     VisibleObjectList* vol)
      throw (TribotsException);

    /**
     * Untersucht, ob es sich bei den zwei übergebenen Übergängen um die 
     * Kanten einer Line handlet. Derzeitige Annahmen:
     *
     * 0. Durch die Übergabe der ScanResults einer bestimmten Farbklasse:
     *    Die Linien sind weiss (COLOR_LINE).
     *
     * 1. Die Farbklasse vor und nach der Linie ist die gleiche.
     *
     * 2. Der Abstand (egozentrische Weltkoordinaten) zwischen den beiden 
     *    Übergängen darf eine Minimalbreite nicht unter- und eine 
     *    Maximalbreite nicht überschreiten (Um zum Beispiel Poster und 
     *    Werbebanner auszusortieren). Hierzu wird die 
     *    Koordinatentransformation Bildkoordinaten-nach-relative 
     *    Weltkoodinaten benötigt.
     *
     * \param transStart Übergang, bei dem die Linie anfangen soll
     * \parma transEnd   Übergang, bei dem die Linie enden soll
     * \returns true, wenn die beiden potentiellen Linienübergänge den
     *          Plausibilitätstest bestanden haben
     */
    bool checkLinePlausibility(const Transition& transStart,
			       const Transition& transEnd,
			       const Frame2d& robot2world) const;

    void setVisualize(Image* vis) { this->vis=vis; }

  protected:
    const ImageWorldMapping* mapping;  ///< Koordinatentransformation
    FieldMapper * fieldmapper;
    double minLineWidth;      ///< unterer Schwellwert für den Abstand
    double maxLineWidth;      ///< oberer Schwellwert für den Abstand
    bool use2Steps;
    bool checkForGreen;
    bool cutOutsideField;
    bool useHalfField;
    bool useMaximalDistance;
    double maximalDistance;
    bool useFieldMapper;
    Image* vis;

    void visualize(const Vec& line) const;
  };

};


#endif
