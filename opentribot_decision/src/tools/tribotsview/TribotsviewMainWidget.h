/****************************************************************************
** Form interface generated from reading ui file 'TribotsviewMainWidget.ui'
**
** Created: Mon Aug 31 14:45:20 2009
**
** WARNING! All changes made in this file will be lost!
****************************************************************************/

#ifndef TRIBOTSVIEWMAINWIDGET_H
#define TRIBOTSVIEWMAINWIDGET_H

#include <qvariant.h>
#include <qpixmap.h>
#include <qmainwindow.h>
#include <qtimer.h>
#include <qmessagebox.h>
#include <qfiledialog.h>
#include <cmath>
#include <deque>
#include <string>
#include <sstream>
#include "CycleContainer.h"
#include "../../Structures/FieldGeometry.h"
#include "../../Fundamental/ConfigReader.h"
#include "SLErrorWidget.h"
#include "ImageviewWidget.h"
#include "../../Fundamental/stringconvert.h"
#include "../components/FieldOfPlay.h"

class QVBoxLayout;
class QHBoxLayout;
class QGridLayout;
class QSpacerItem;
class QAction;
class QActionGroup;
class QToolBar;
class QPopupMenu;
namespace TribotsTools {
class FieldOfPlay;
}
namespace TribotsTools {
class RotateView;
}
class QSplitter;
class QGroupBox;
class QPushButton;
class QSlider;
class QTextEdit;
class QLabel;
class QLineEdit;

class TribotsviewMainWidget : public QMainWindow
{
    Q_OBJECT

public:
    TribotsviewMainWidget( QWidget* parent = 0, const char* name = 0, WFlags fl = WType_TopLevel );
    ~TribotsviewMainWidget();

    QSplitter* splitter4;
    QGroupBox* groupBox1;
    QPushButton* goto_begin_button;
    QPushButton* goto_end_button;
    QSlider* cycle_slider;
    QSplitter* splitter2;
    TribotsTools::FieldOfPlay* field_of_play;
    QTextEdit* textfeld;
    QTextEdit* textEditIncoming;
    QTextEdit* textEditOutgoing;
    QGroupBox* groupBox11;
    QLabel* textLabel2;
    QLineEdit* cycle_num;
    QLineEdit* prog_time;
    QLabel* textLabel1;
    QLabel* textLabel1_7;
    QLineEdit* refereeState;
    QLineEdit* ball_known;
    QLabel* textLabel3;
    QLabel* textLabel2_2;
    QLineEdit* robot_velocity;
    QTextEdit* textEditBehavior;
    TribotsTools::RotateView* rotate_view;
    QSlider* display_rate;
    QLineEdit* lineEditImageSource;
    QLineEdit* lineEditPlayertype;
    QMenuBar *MenuBar;
    QPopupMenu *Programm;
    QPopupMenu *Abspielen;
    QPopupMenu *Extras;
    QPopupMenu *Lesezeichen;
    QPopupMenu *Einstellungen;
    QToolBar *FileToolbar;
    QToolBar *PlayToolbar;
    QToolBar *ExtrasToolbar;
    QAction* fileExitAction;
    QAction* PlayFwd;
    QAction* PlayFastFwd;
    QAction* PlayFastRew;
    QAction* StopPlay;
    QAction* PlayStepRew;
    QAction* PlayStepFwd;
    QAction* Revert;
    QAction* Reload;
    QAction* PlayRew;
    QAction* ReplaceCycleInfo;
    QAction* DisplayImages;
    QAction* LoadImages;
    QAction* AddLogfile;
    QAction* NextRefState;
    QAction* PrevRefState;
    QAction* toggleBookmarkAction;
    QAction* gotoNextBookmarkAction;
    QAction* gotoPrevBookmarkAction;
    QAction* clearBookmarksAction;
    QAction* useColoredGoalsAction;
    QAction* useSyncSignalsAction;
    QAction* useGreyRobotsAction;

public slots:
    virtual void init_field_and_streams( const std::deque<std::string> & info_praefix, const std::string & config_file, bool use_sync_signals1, bool use_colored_goals1, bool use_attr );
    virtual void cycleChanged();
    virtual void sl_pos_changed();
    virtual void displayChanged();
    virtual void prevRefStateCycle();
    virtual void nextRefStateCycle();
    virtual void nextCycle();
    virtual void prevCycle();
    virtual void play_on();
    virtual void start_play();
    virtual void start_rew();
    virtual void start_ffw();
    virtual void start_frew();
    virtual void stop_play();
    virtual void setCycleNum();
    virtual void setTime();
    virtual void toogleImageView( bool b );
    virtual void change_display_frequency( int v );
    virtual void cycle_slider_moved( int i );
    virtual void cycle_slider_value_changed( int i );
    virtual void fileExit();
    virtual void revert_file();
    virtual void reload_file();
    virtual void loadImages();
    virtual void showSLError( Tribots::Vec p1, Tribots::Vec p2 );
    virtual void replaceCycleInfo();
    virtual void unresolvedKeyPressEvent( QKeyEvent * event );
    virtual void show();
    virtual void displayStatusMessage( QString s );
    virtual void refrobotChanged();
    virtual void toggleColoredGaols( bool b );

protected:
    int wait_msec;
    int play_mode;
    ImageviewWidget* imageviewDialog;
    int newVariable;

    QGridLayout* TribotsviewMainWidgetLayout;
    QGridLayout* groupBox1Layout;
    QGridLayout* layout4;
    QHBoxLayout* layout2;
    QGridLayout* groupBox11Layout;

protected slots:
    virtual void languageChange();

    virtual void goto_start();
    virtual void goto_end();
    virtual void loadAdditionalLogfile();
    virtual void keyPressEvent( QKeyEvent * ev );
    virtual void buildCycleInfo();
    virtual void toggleBookmark();
    virtual void gotoNextBookmark();
    virtual void gotoPrevBookmark();
    virtual void clearBookmarks();


private:
    bool use_sync_signals;
    TribotsTools::CycleInfo cycle_info;
    std::deque<TribotsTools::CycleContainer*> cycle_container;
    QTimer play_control;
    TribotsTools::SLErrorWidget* slwidget;
    std::string old_behavior;
    std::vector<unsigned long int> bookmarks;
    bool use_attributes;

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
    QPixmap image11;
    QPixmap image12;
    QPixmap image13;
    QPixmap image14;
    QPixmap image15;
    QPixmap image16;

    virtual Tribots::FieldGeometry read_field_geometry();
    virtual void synchronize();

private slots:
    virtual void toggleGreyRobots( bool b );
    virtual void toggleSyncSignals( bool b );

};

#endif // TRIBOTSVIEWMAINWIDGET_H
