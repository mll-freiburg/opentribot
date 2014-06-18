/****************************************************************************
** Form interface generated from reading ui file 'mainWidget.ui'
**
** Created: Mon Aug 31 14:47:02 2009
**
** WARNING! All changes made in this file will be lost!
****************************************************************************/

#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <qvariant.h>
#include <qpixmap.h>
#include <qmainwindow.h>
#include "qfile.h"

class QVBoxLayout;
class QHBoxLayout;
class QGridLayout;
class QSpacerItem;
class QAction;
class QActionGroup;
class QToolBar;
class QPopupMenu;
class QLabel;
class QLineEdit;
class QPushButton;
class QSplitter;
class QListView;
class QListViewItem;
class QTextEdit;
namespace Tribots {class ConfigReader;};
class ConfigReader;
class QFile;
class TribotsSyntax;

class mainWidget : public QMainWindow
{
    Q_OBJECT

public:
    mainWidget( QWidget* parent = 0, const char* name = 0, WFlags fl = WType_TopLevel );
    ~mainWidget();

    QLabel* textLabel1;
    QLineEdit* lineEditMainConfig;
    QPushButton* loadButton;
    QSplitter* splitter2;
    QListView* listView1;
    QTextEdit* textEditor;
    QPushButton* pushButtonCommentOutInLine;
    QPushButton* saveFileButton;
    QMenuBar *MenuBar;
    QPopupMenu *fileMenu;
    QPopupMenu *Edit;
    QToolBar *Toolbar;
    QToolBar *Toolbar_2;
    QAction* fileNewAction;
    QAction* fileOpenAction;
    QAction* fileSaveAction;
    QAction* fileSaveAsAction;
    QAction* filePrintAction;
    QAction* fileExitAction;
    QAction* editUndoAction;
    QAction* editRedoAction;
    QAction* editCutAction;
    QAction* editCopyAction;
    QAction* editPasteAction;
    QAction* editFindAction;
    QAction* helpContentsAction;
    QAction* helpIndexAction;
    QAction* helpAboutAction;

    virtual void updateListView( Tribots::ConfigReader * cr );

public slots:
    virtual void slotLoadMainConfig( const QString & fileName );
    virtual void slotFileOpen();
    virtual void chooseOpenFile();
    virtual void slotClickedListViewItem( QListViewItem * lv );
    virtual void slotFileSave();
    virtual void aboutSlot();
    virtual void slotReloadMainConfig();
    virtual void slotAskForSave();
    virtual void commentLine();
    virtual void closeClicked();

protected:
    QFile usedFile;
    TribotsSyntax * tribotssyntax;

    QGridLayout* mainWidgetLayout;
    QHBoxLayout* layout3;
    QVBoxLayout* layout5;
    QHBoxLayout* layout4;
    QSpacerItem* spacer1;

protected slots:
    virtual void languageChange();

private:
    QPixmap image0;
    QPixmap image1;
    QPixmap image2;
    QPixmap image3;
    QPixmap image4;
    QPixmap image5;
    QPixmap image6;
    QPixmap image7;
    QPixmap image8;
    QPixmap image9;
    QPixmap image10;

    void init();
    void destroy();
    virtual std::string get_filename( const std::string & src );
    virtual std::string get_pathname( const std::string & src );

};

#endif // MAINWIDGET_H
