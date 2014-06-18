
#ifndef Tribots_DynamicSlidingWindowBallFilter3D_h
#define Tribots_DynamicSlidingWindowBallFilter3D_h

#include "BallFilter.h"
#include "../../Fundamental/ConfigReader.h"
#include "../../Fundamental/DynamicRingBuffer.h"
#include "../../Fundamental/RingBuffer.h"
#include "../../Fundamental/Hysteresis.h"

namespace Tribots {

  /** eine Klasse, die den BallRaised-Mechanismus des Ballfilters kapselt */
  class BallRaisedMechanism {
    RingBuffer<Vec> positions;  ///< alte Positionen, da bei raised nicht die letzte bekannte Position sondern eine aeltere zurueckgeliefert wird. Konvention: positions.get() liefert die aelteste Position, positions.get(-1) die neueste
    Hysteresis raisedHysteresis;
  public:
    /** Konstruktor, sinnvoller Weise muss noch setField() und ggf. setHysteresis() aufgerufen werden */
    BallRaisedMechanism () throw () : positions(6), raisedHysteresis (5, 7, false) {;}
    /** die beiden Schwellwerte der Hysterese setzen */
    void setHysteresis (double lt, double ut) throw () { raisedHysteresis.setThreshold (lt, ut); }
    /** raised explizit setzen/loeschen */
    void set (bool s) throw () { raisedHysteresis.set(s); }
    /** raised abfragen */
    bool raised () const throw () { return raisedHysteresis.get(); }
    /** Position fuer raised abfragen */
    Vec raisedPosition () const throw () { return positions.get(); }
    /** aktualisieren mit aktueller Ballposition und Ballgeschwindigkeit */
    void update (Vec pos, double velocity) throw () { positions.get()=pos; positions.step(1); raisedHysteresis.update (velocity); }
  };


  /** eine Beobachtung des Balles */
  struct BallObservation {
    Vec3D pos;  ///< Position
    Time time;  ///< Zeitstempel
    bool true3d;  ///< echte 3D-Messung?
    Vec robot;  ///< Roboterposition zum Zeitpunkt der Messung
  };

  /** eine Klasse, die ein lineares Bewegungsmodell schaetzt und praediziert */
  class LinearBallMotionModel2D {
    Vec referencePosition;
    Vec referenceVelocity;
    Time referenceTime;  ///< Zeitpunkt der neuesten Beobachtung
    /** interne Schaetzfunktion. Liefert Anzahl beruecksichtigter Beobachtungen */
    static unsigned int estimate (Time& reftime, Vec& refpos, Vec& refvel, const std::vector<BallObservation>& observations, bool use2d, bool use3d) throw ();
  public:
    /** liefere die praedizierte Ballposition zum Zeitpunkt t */
    Vec getPosition (Time t) const throw () { return referencePosition+t.diff_msec(referenceTime)*referenceVelocity; }
    /** liefere die Ballposition zum Referenzzeitpunkt (der letzten Beobachtung) */
    Vec getReferencePosition () const throw () { return referencePosition; }
    /** setze die Ballposition zum Referenzzeitpunkt (der letzten Beobachtung) */
    void setReferencePosition (Vec p) throw () { referencePosition=p; }
    /** liefere die praedizierte Ballgeschwindigkeit zum Zeitpunkt t */
    Vec getVelocity (Time) const throw () { return referenceVelocity; }
    /** liefere die Ballgeschwindigkeit zum Referenzzeitpunkt (der letzten Beobachtung) */
    Vec getReferenceVelocity () const throw () { return referenceVelocity; }
    /** liefere den Referenzzeitpunkt, d.h. den Zeitpunkt der neuesten Beobachtung */
    Time getReferenceTime () const throw () { return referenceTime; }

    /** Das Bewegungsmodell initialisieren mit Anfangsposition, -geschwindigkeit und -referenzzeitpunkt */
    void set (Vec p, Vec v, Time t) throw () { referencePosition=p; referenceVelocity=v; referenceTime=t; }
    /** Das Bewegungsmodell schaetzen aus einer Liste von Beobachtungen. Bei Erfolg wird true zurueckgegeben.
        Mit use2d und use3d kann festgelegt werden, ob 2D- bzw. 3D-Beobachtungen verwendet werden sollen oder nicht.
        Bei Misserfolg bleibt das bisherige Bewegungsmodell unveraendert. */
    bool reestimate (const std::vector<BallObservation>& observations, bool use2d=true, bool use3d=true) throw ();
    /** das Bewegungsmodell schaetzen mit EM-Algorithmus aus 2D- und 3D-Positionen;
        die 2D-Positionen in observations werden an das Bewegungsmodell angepasst.
        Argument robot ist die Roboterposition. */
    bool reestimateEM (const std::vector<BallObservation>& observations, Vec robot) throw ();
  };

  /** ein Bewegungsmodell fuer die Ballhoehe */
  class BallMotionModelZ {
    double referencePosition;
    double referenceVelocity;
    Time referenceTime;  ///< Zeitpunkt der neuesten Beobachtung

    void predict (Time t) throw ();  ///< fuellen von predictPosition, predictVelocity, predictTime
    double predictPosition;
    double predictVelocity;
    Time predictTime;
  public:
    /** liefere die praedizierte Ballposition zum Zeitpunkt t */
    double getPosition (Time t) const throw () {  const_cast<BallMotionModelZ*>(this)->predict(t); return predictPosition; }
    /** liefere die Ballposition zum Referenzzeitpunkt (der letzten Beobachtung) */
    double getReferencePosition () const throw () { return referencePosition; }
    /** setze die Ballposition zum Referenzzeitpunkt (der letzten Beobachtung) */
    void setReferencePosition (double p) throw () { referencePosition=p; }
    /** liefere die praedizierte Ballgeschwindigkeit zum Zeitpunkt t */
    double getVelocity (Time t) const throw () { const_cast<BallMotionModelZ*>(this)->predict(t); return predictVelocity; }
    /** liefere die Ballgeschwindigkeit zum Referenzzeitpunkt (der letzten Beobachtung) */
    double getReferenceVelocity () const throw () { return referenceVelocity; }
    /** liefere den Referenzzeitpunkt, d.h. den Zeitpunkt der neuesten Beobachtung */
    Time getReferenceTime () const throw () { return referenceTime; }
    /** liefere den erwarteten Zeitpunkt, zu dem der Ball den Boden beruehren wird */
    Time getTimeTouchingGround () const throw ();
    /** liefere den hoechsten zu erwartenden Punkt */
    double getHighestPoint () const throw ();

    /** Das Bewegungsmodell initialisieren mit Anfangsposition, -geschwindigkeit und -referenzzeitpunkt */
    void set (double p, double v, Time t) throw () { referencePosition=p; referenceVelocity=v; referenceTime=t; }
    /** Das Bewegungsmodell schaetzen aus einer Liste von Beobachtungen. Bei Erfolg wird true zurueckgegeben.
        Mit use2d und use3d kann festgelegt werden, ob 2D- bzw. 3D-Beobachtungen verwendet werden sollen oder nicht. 
        Bei Misserfolg bleibt das bisherige Bewegungsmodell unveraendert. */
    bool reestimate (const std::vector<BallObservation>& observations) throw ();
  };

  /** Klasse DynamicSlidingWindowBallFilter3D modelliert eine einfache Ballverfolgung und 
      Geschwindigkeitsschaetzung; Das Verfahren basiert auf der Bildung eines
      kleinste-Quadrate-Schaetzers ueber die letzten n gesehenen Ballpositionen
      bei linearem Bewegungsmodell (gleichfoermige Bewegung); n wird automatisch angepasst */
  class DynamicSlidingWindowBallFilter3D : public BallFilter {
  public:
    /** erzeuge BallFilter; lese Parameter aus arg1 */
    DynamicSlidingWindowBallFilter3D (const ConfigReader&) throw (std::bad_alloc);
    /** Destruktor */
    ~DynamicSlidingWindowBallFilter3D () throw ();
    /** uebergebe gemessene Position an Filter;
        arg1=Position des Balls in Weltkoordinaten (z-Koordinaten <-100 werden als unbekannt interpretiert),
        arg2=Zeitpunkt der Messung,
        arg3=Position des Roboters in Weltkoordinaten */
    void update (const Vec3D, const Time, const Vec) throw ();
    /** uebergebe neue Bildinformation an Filter, 
        arg1=gesehener Ballposition,
        arg2=Roboterposition und -ausrichtung zum Zeitpunkt der Bildinformation
        ret = wurde tatsaechliche eine Aktualisierung vorgenommen? */
    bool update (const VisibleObjectList&, const RobotLocation&) throw ();
    /** Kommunizierten Ball uebergeben (in Weltkoordinaten) */
    void comm_ball (Vec) throw ();
    /** liefere mutmasliche Position und Geschwindigkeit zum Zeitpunkt arg1 */
    BallLocation get (const Time) const throw ();

  private:
    BallRaisedMechanism raised;   ///< Mechanismus, um raised zu verwalten
    LinearBallMotionModel2D ballMotion2d;  ///< das 2D-Bewegungsmodell
    BallMotionModelZ ballMotionZ;  ///< das Bewegungsmodell fuer die Ballhoehe
    BallObservation latestObservation;  ///< die letzte gueltige Beobachtung (gueltig=innerhalb des Feldes)
    BallObservation ballCommunicated;  ///< der letzte kommunizierte Ball

    DynamicRingBuffer<BallObservation> observations;   ///< Ringpuffer mit bisherigen Beobachtungen

    unsigned int maxBufferSize;  ///< maximale Groesse des Ringpuffers
    unsigned int minBufferSize;  ///< minimale Groesse des Ringpuffers

    bool largeErrorObserved;  ///< war der Fehler der letzten Beobachtung groesser als erlaubt?
    bool lowBall;  ///< Bewegungsmodell wurde nur aus 2D-Beobachtungen geschaetzt
    double maxError;   ///< maximal erlaubter Fehler

    /** Element in Ringpuffer einfuegen/ersetzen */
    void addToRingbuffer (const BallObservation& obs);
    /** Den Ringbuffer durch Entfernen der aeltesten Positionen auf Groesse n (n>=1) bringen */
    void rescaleRingBuffer (unsigned int n) throw ();
    /** Prueft, ob p sich innerhalb des Spielfeldes (inclusive Rand und Toleranzbereich) befindet */
    bool insideField (Vec p) const throw ();
  };

}

#endif
