
#ifndef _TribotsTools_VisionToolMainWindow_h_
#define _TribotsTools_VisionToolMainWindow_h_

#include <QtGui/QTabWidget>
#include <QtGui/QMainWindow>

#include "../Logic/VisionToolImageSource.h"
#include "VisionToolWidget.h"
#include <vector>


namespace TribotsTools {

  class VisionToolMainWindow : public QMainWindow {
    Q_OBJECT

  public:
    VisionToolMainWindow (VisionToolImageSource& is, Tribots::ConfigReader& cfg, QWidget* parent =0, Qt::WindowFlags flags =0);
    ~VisionToolMainWindow ();
    void loop ();

  signals:
    void widgetClosed();

  public slots:
    void resizeEvent ( QResizeEvent * );
    void closeEvent ( QCloseEvent * );
    void tabChanged (int);
    void saveExit ();

  private:
    VisionToolImageSource& imageSource;
    Tribots::ConfigReader& config;

    QTabWidget *tabWidget;
    std::vector<VisionToolWidget*> tabs;
    unsigned int activeWindow;

    bool continueLoop;

    void saveConfig ();  ///< ConfigReader-Inhalt speichern in temporaerer Datei (fallback)
    void addTab (const char*, VisionToolWidget*);  ///< ein Tab hinzufuegen
  };

}

#endif
