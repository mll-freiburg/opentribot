
#ifndef _TribotsTools_TribotsviewMainWidget_h_
#define _TribotsTools_TribotsviewMainWidget_h_

#include "UI/TribotsviewMainWidget.h"

#include <QtCore/QTimer>
#include <QtGui/QMessageBox>
#include <QtGui/QFileDialog>

#include <deque>

#include "SLErrorWidget.h"
#include "ImageviewWidget.h"
#include "CycleContainer.h"
#include "../../../Structures/FieldGeometry.h"
#include "../../../Fundamental/stringconvert.h"
#include "../../../Fundamental/ConfigReader.h"

namespace TribotsTools {

  class TribotsviewMainWidget : public QMainWindow, private Ui::TribotsviewMainWidget {
    Q_OBJECT

  public:
    TribotsviewMainWidget (QWidget* =0, Qt::WindowFlags =0);
    ~TribotsviewMainWidget ();

  public slots:
    /** Arg1: Logfile-Basisnamen (z.B. wminfo), Arg2: Cfg-Datei, Arg3: Synchronisationssignale verwenden (true) oder Systemzeitstempel (false) */
    void init_field_and_streams (const std::deque<std::string>&, const std::string&, bool);

    void nextCycle ();  ///< Einzelschritt +1
    void prevCycle ();  ///< Eimzelschritt -1
    void play_on ();  ///< einen Schritt automatisch weiterschalten

    void start_play (); ///< vorwaerts abspielen
    void start_rew (); ///< rueckwaerts abspielen
    void start_ffw ();  ///< schnell vorwaerts
    void start_frew (); ///< schnell rueckwaerts
    void stop_play ();  ///< anhalten

    void setCycleNum ();  ///< Iteration direkt setzen
    void setTime ();  ///< Programmzeit direkt setzen
    void goto_start ();  ///< an Start springen
    void goto_end ();  ///< an Ende springen

    void toggleImageView (bool);  ///< Kamerabild anzeigen an/aus
    void change_display_frequency (int);  ///< Anzeigefrequenz aendern
    void cycle_slider_moved (int);  ///< Zyklus ueber Slider einstellen
    void cycle_slider_value_changed (int);  ///< Zyklus ueber Slider einstellen

    void fileExit ();  ///< Programm beenden
    void revert_file ();  ///< Logdateien erneut laden
    void reload_file ();  ///< neue Logfiles laden
    void loadAdditionalLogfile ();  ///<zusaetzliches Logfile laden
    void loadImages ();  ///< Kamerabilder laden
    void showSLError (Tribots::Vec, Tribots::Vec);  ///< SL-Fehler darstellen, Arg1, Arg2 sind die Eckpunkte des Ausschnitts
    void replaceCycleInfo ();  ///< im Container gespeicherte Infos ersetzen durch aktuelle Infos (insbes. nach manueller Repositionierung)

    void keyPressEvent (QKeyEvent*);  ///< Tastendruck
    void unresolvedKeyPressEvent(QKeyEvent*);   ///< Tastendruck in Teilfenster
    void show ();

    void displayStatusMessage (QString);  ///< Nachricht auf Statuszeile anzeigen
    void sl_pos_changed ();
    void refrobotChanged ();

  protected slots:
    void showEvent ( QShowEvent * );

  private:
    void cycleChanged ();
    void displayChanged ();
    void buildCycleInfo ();
    Tribots::FieldGeometry read_field_geometry ();
    void synchronize ();


    bool use_synch_signals;
    int wait_msec;
    int play_mode;

    ImageviewWidget* imageviewDialog;
    SLErrorWidget* slwidget;

    CycleInfo cycle_info;
    std::deque<CycleContainer*> cycle_container;

    QTimer play_control;

    bool first_show;
  };

}

#endif
