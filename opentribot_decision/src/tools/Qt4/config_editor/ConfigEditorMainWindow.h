
#ifndef _TribotsTools_ConfigEditorMainWindow_h_
#define _TribotsTools_ConfigEditorMainWindow_h_

#include "UI/ConfigEditorMainWindow.h"
#include <QtGui/QMainWindow>
#include <QtCore/QString>
#include <QtGui/QListView>
#include <QtCore/QFile>
#include <string>
#include "../../../Fundamental/ConfigReader.h"
#include "TribotsSyntax.h"

namespace TribotsTools {

  class ConfigEditorMainWindow : public QMainWindow, private Ui::ConfigEditorMainWindow {
    Q_OBJECT

  public:
    ConfigEditorMainWindow (QWidget* =0, Qt::WindowFlags =0);

  public slots:
    virtual void slotLoadMainConfig(const QString&);
    virtual void chooseOpenFile();
    virtual void updateListView(Tribots::ConfigReader*);
    virtual void slotClickedListViewItem(const QString&);
    virtual void slotFileSave();
    virtual void aboutSlot();
    virtual void slotReloadMainConfig();

  private:
    virtual std::string get_filename(const std::string&);
    virtual std::string get_pathname(const std::string&);

    TribotsSyntax* tribotssyntax;
    QFile usedFile;
  };

}

#endif
