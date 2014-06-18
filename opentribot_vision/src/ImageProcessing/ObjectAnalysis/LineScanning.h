#ifndef _linescanning_h_
#define _linescanning_h_

#include <vector>
#include "../../Structures/TribotsException.h"
#include "../Formation/Image.h"
#include "ScanLines.h"
#include "ColorClasses.h"
#include "LineFilter.h"

namespace Tribots {

  /**
   * Diese Stuktur enthält Informationen über im Bild gefundene Farbübergänge.
   * Jede Transition ist entweder vom Typ START oder vom Typ END. Der Typ
   * gibt an, ob es sich - in der Richtung, mit der eine Scanlinie 
   * abgeschritten wurde - um den Anfang oder das Ende einer Region handelt.
   */
  struct Transition {
    int type;         ///< START: start of the region, END: end of the region
    int from;         ///< class of the first pixel's color
    int to;           ///< class of the second pixel's color
    int twoStep;      ///< START: class of pixel two steps back, END: class of pixel two Steps ahead
    Vec fromPos;      ///< x-y-coordinates of the first pixel (image coords)
    Vec toPos;        ///< x-y-coordinates of the second pixel (image coords)
    bool virt;        ///< whether or not this transition is a virtual one

    /**
     * Konstruiert eine Transition.
     *
     * \param type    gibt den Typ an (START oder END)
     * \param from    Farbklasse des letzten Pixels vor dem Übergang
     * \param to      Farbklasse des ersten Pixels nach dem Übergang
     * \param fromPos Koordinaten des letzten Pixels vor dem Übergang
     * \param toPos   Koordinaten des letzten Pixels nach dem Übergang
     */
    inline Transition(int type, int from, int to, int twoStep,
		      const Vec& fromPos, const Vec& toPos, bool virt=false);
    enum { START=0, END };
  };
    
  /**
   * Diese Struktur enthält die Resultate des Scanvorgangs für eine einzelne
   * Farbklasse. Gespeichert werden (je nach Einstellung in der ColorClassInfo)
   * Übergänge und Punkte (Koordinaten der Pixel dieser Farbklasse).
   */
  struct ScanResult {
    int id;                     ///< Farbklasse, auf die sich bezogen wird
    std::vector <Transition> transitions; ///< gefundene Farbübergänge
    std::vector <Vec> points;   ///< gefundene Punkte dieser Farbklasse

    /**
     * Konstruktor, der ein Resultatobjekt für die angegebene Farbklasse
     * erzeugt.
     */
    inline ScanResult(int id);  
    /**
     * löscht alle gefundenen Punkte und Übergänge
     */
    void clear(); 
  };

  class ScanResultList {
  public:
    std::vector<ScanResult*> results;  ///< Liste von ScanResult Strukturen
    
    /**
     * Konstruktor, der eine Liste für n Klassen anlegt.
     */
    ScanResultList(int n);

    /**
     * Destruktor.
     */
    ~ScanResultList();

    /**
     * löscht alle gefundenen Punkte und Übergänge
     */
    void clear();
  };
      

  class LineScanner {
  public:
    /**
     * Initialisiert einen Linienscanner mit einer Menge von Linien
     * und für die übergebenen Farbklasseninformationen.
     *
     * \param lines Scanlinien, Instanz geht in den Besitz des Linienscanners
     *              über und kann nicht mehr für einen anderen Scanner 
     *              verwendet werden.
     * \param colClasses Informationen über die zu suchenden Farbklassen.
     *                   Objekt geht nicht in den Besitz des Linienscanners
     *                   über und muss daher extern gelöscht werden - 
     *                   allerdings nicht vor dem Linienscanner!
     */
    LineScanner(ScanLines* lines, 
		const ColorClassInfoList* colClasses,
		bool _scanBlackPixelNeighbourhood = false );
    ~LineScanner();

    /**
     * Scannt ein Bild und zeichnet die gefunden Übergänge und Pixel in der
     * übergebenen ScanResultList auf. Die gefundenen Informationen werden
     * an die bereits in der ScanResultList enthaltenen Daten angehängt!
     */
    void scan(const Image& image, ScanResultList* results)
      throw (TribotsException);

    /** 
     * Übergibt das Bild, in das die beim nächsten Aufruf von scan 
     * gefundenen Übergänge eingezeichnet werden sollen.
     */
    void setVisualize(Image* vis) { this->vis = vis; }


    LineFilter *linefilter;
    bool use_linefilter;
  protected:
    ScanLines* lines;    ///< Liste von Scanlinien 
    const ColorClassInfoList* colClasses;///< Zeiger zur Farbklasseninformation
    Image* vis;
    bool scanBlackPixelNeighbourhood;

    void visualize(const ScanResultList* results);
  };


  // inline /////////////////////////////////////////////////////////

  Transition::Transition(int type, int from, int to, int twoStep,
			 const Vec& fromPos, const Vec& toPos, bool virt)
    : type(type), from(from), to(to), twoStep(twoStep), fromPos(fromPos), 
      toPos(toPos), virt(virt)
  {}
  
  ScanResult::ScanResult(int id)
    : id(id)
  {}
   
}

#endif
