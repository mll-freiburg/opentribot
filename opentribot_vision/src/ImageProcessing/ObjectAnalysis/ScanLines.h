#ifndef _scanlines_h_
#define _scanlines_h_

#include <vector>
#include "../../Fundamental/Vec.h"
#include "../../Fundamental/geometry.h"
#include "../PixelAnalysis/RobotMask.h"


namespace Tribots {

  /**
   * Struktur enthält eine Liste von Scanlinien, die einem LineScanner 
   * übergeben werden können.
   */
  class ScanLines {
  public:
    /**
     * Konstruktor, der eine Menge von Scanlinien erzeugt, die Radial
     * um den angegebenen Mittelpunkt angeordnet sind.
     *
     * \param robotMask Die Robotermaske
     * \param middle Koordinaten des Mittelpunktes, um den die Linien 
     *               angeordnet werden sollen
     * \param innerRadius Der innere Radius gibt an, in welchem Abstand vom 
     *                    Mittelpunkt die Scanlinien beginnen sollen.
     * \param outerRadius  Der äußere Radius gibt an, in welchem Abstand vom
     *                    Mittelpunkt die Scanlinien enden sollen.
     * \param width Gesamtbreite des Bildes, an der die Scanlinien 
     *              abgeschnitten (clipping) werden sollen.
     * \param height Gesamthöhe des Bildes, an der die Scanlinien abgeschnitten
     *               (clipping) werden sollen.
     * \param n Anzahl der "Scanliniensegmente". Jedes Scanliniensegment 
     *          besteht aus mehreren, auf eine bestimmte Weise 
     *          angeordneten Scanlinien, die n-Mal wiederholt werden.
     * \param kaestchen wenn true, werden zusaetzlich zu den radialen Scanlinien 
     *          5 Kaestchen um den Mittelpunkt eingebaut
     */
    ScanLines(const RobotMask* robotMask, const Vec& middle, int innerRadius, int outerRadius, 
	      int width, int height, int n, bool kaestchen = true);

    /** 
     * Konstruktor, der eine einzelne Scanlinie erzeugt.
     */
    ScanLines(const RobotMask* robotMask, const Vec& start, const Vec& end, 
              int width=0, int height=0);
    
    std::vector<LineSegment> scanLines; ///< Liste von Scanlinien
    
    static ScanLines* createForDirectedCamera(const RobotMask* robotMask,
                                              int imageWidth, int imageHeight, 
                                              int nScanLines);
    static ScanLines* createForOmniCamera(const RobotMask* robotMask, 
                                          const Vec& middle, 
                                          int innerRadius, int outerRadius, 
                                          int width, int height, 
                                          int n, bool kaestchen = true);
  protected:
    ScanLines(); 
    void insertScanLine(const RobotMask* robotMask, const LineSegment& line, int width, int height);
  };

}

#endif
