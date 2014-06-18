
#ifndef _TribotsTools_SLErrorWidget_h_
#define _TribotsTools_SLErrorWidget_h_

#include <qwidget.h>
#include <qpushbutton.h>
#include "../components/CycleInfo.h"
#include "../../WorldModel/SL/FieldLUT.h"
#include "../../WorldModel/SL/VisualPositionOptimiser.h"
#include "../../Fundamental/ConfigReader.h"


namespace TribotsTools {

  /** Klasse, um eine SL-Fehlerlandschaft zu zeichnen.
      Arbeitet mit Seiteneffekten auf ein CycleInfo-Objekt */
  class SLErrorWidget : public QWidget {
    Q_OBJECT
  private:
    static const double cellsize; // Zellengroese in mm

    unsigned char** error_array;  // das Fehlerarray
    unsigned int width;           // Fehlerarray, x-Dimension
    unsigned int height;          // Fehlerarray, y-Dimension

    Tribots::Vec center;          // Mittelpunkt des Spielfeldausschnitts
    double xext;                  // Breite des Ausschnitts in x-Richtung
    double yext;                  // Breite des Ausschnitts in y-Richtung
    int own_half;                 // +/-1 je nach Orientierung des Feldes

    double scale;                 // Skalierung des Bildes
    double offset_x;              // Offset eigentliches Bild
    double offset_y;              // Offset eigentliches Bild

    unsigned int best_i;          // Zeilennummer des besten Eintrags
    unsigned int best_j;          // Spaltennummer des besten Eintrags
    unsigned int best_phi;        // beste Orientierung in Grad

    Tribots::Vec optimising_pos;      // beste Position aus Optimierung der gegenwaertigen Position
    Tribots::Angle optimising_angle;  // beste Ausrichtung aus Optimierung der gegenwaertigen Position

    CycleInfo* ci;                // die aktuelle Zyklusinfo, wird von SLErrorWidget nicht besessen!
    Tribots::FieldLUT* fl;        // die Feld-Nachschlagetabelle
    double probability_line_min;  // die entsprechende Groesse aus CondensationFilter
    double probability_line_sdev; // die entsprechende Groesse aus CondensationFilter

    const Tribots::FieldGeometry* fg;   // die Feldgeometrie

    Tribots::VisualPositionOptimiser* optimiser;  // Optimierer, um von gesetzter Position optimale Position iterativ zu finden
    double optimiser_error_param;                 // Fehlerparameter fuer Optimierer
    unsigned int num_opt_iter;                    // Anzahl Iterationen fuer Optimierer

    QPushButton* quit_button;
    QPushButton* revert_button;
    QPushButton* revert_opt_button;

    double calculate_fitness (double&, Tribots::Vec);  // berechne die best Guete als Wert zwischen 0 und 255 an pos arg2; arg1 ist Rueckgabe des besten Winkel in Grad
  public:
    /** Konstruktor wie QWidget 
	Argumente wie normales Widget */
    SLErrorWidget ( QWidget * = 0, const char * = 0, WFlags = 0 );
    /** Destruktor */
    ~SLErrorWidget ();

    /** initialisierung der FieldLUT und der Fehler-Parameter */
    void init_pointers (const Tribots::ConfigReader&, const Tribots::FieldGeometry&);

    /** Fehlerlandschaft aktualisieren:
	arg1 = Zyklus-Info
	arg2, arg3 = Ecken des zu untersuchenden Bildausschnitts 
	arg4 = 'own_half', gespiegelt (-1) oder nicht (1) */
    void update_error (CycleInfo&, Tribots::Vec, Tribots::Vec, int);

  protected:
    /** Zeichnen des Widgets */
    void paintEvent(QPaintEvent *);
    /** Groesse veraendert -> Buttons neu anordnen */
    void resizeEvent ( QResizeEvent * );
    /** Mausklick zum Versetzen des Roboters */
    void mousePressEvent(QMouseEvent*);
    /** Umrechnen von Koordinaten */
    Tribots::Vec widget2field (Tribots::Vec);
    /** Umrechnen von Koordinaten */
    Tribots::Vec field2widget (Tribots::Vec);
    
  public slots:
    /** Fehlerlandschaft neu berechnen ohne Bildauscchnitt zu veraendern */
    void update_error ();
    /** Optimierungsposition neu berechnen */
    void update_optimising_pos ();

  signals:
    /** Roboterposition veraendert -> FieldOfPlay muss neu gezeichnet werden */
    void robot_update ();
  };

}

#endif
