
#ifndef _TribotsTools_VisionToolWidget_h_
#define _TribotsTools_VisionToolWidget_h_

#include <QtGui/QWidget>
#include <QtGui/QStatusBar>
#include "../../../..//Fundamental/ConfigReader.h"

namespace TribotsTools {

  /** Allgemeine Oberklasse fuer VisionTool Widgets. Stellt die Schnittstelle mit den drei
      Methoden start(), loop() und stop() bereit */
  class VisionToolWidget : public QWidget {
    Q_OBJECT

  public:
    VisionToolWidget (Tribots::ConfigReader& cfg, QStatusBar& stb, QWidget* parent =0, Qt::WindowFlags flags =0) : QWidget (parent,flags), config(cfg), statusBar(stb) {;}
    ~VisionToolWidget () {;}

    virtual void start () {;} ///< Die Aktivitaeten des Widget starten
    virtual void stop () {;} ///< Die Aktivitaeten des Widgets beenden/unterbrechen, Ressourcen freigeben
    virtual void loop () {;} ///< regelmaessig aufgerufene Routine zwischen start und stop, innerhalb der irgendwas gemacht werden kann, z.B neues Bild laden

    protected:
      Tribots::ConfigReader& config;
      QStatusBar& statusBar;
  };

}

#endif
