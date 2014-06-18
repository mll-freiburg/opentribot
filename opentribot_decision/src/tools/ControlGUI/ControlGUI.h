/****************************************************************************
** Form interface generated from reading ui file 'ControlGUI.ui'
**
** Created: Mon Aug 31 14:47:13 2009
**
** WARNING! All changes made in this file will be lost!
****************************************************************************/

#ifndef CONTROLGUI_H
#define CONTROLGUI_H

#include <qvariant.h>
#include <qpixmap.h>
#include <qmainwindow.h>
#include <qtimer.h>
#include <qtextedit.h>
#include <qfileinfo.h>
#include <qdir.h>
#include "../../Fundamental/ConfigReader.h"
#include "ControlGUIState.h"
#include "../../Communication/TribotsUDPCommunication.h"
#include "DeactivateWidget.h"

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
class MonoLED;
}
class QGroupBox;
class QPushButton;
class QComboBox;
class QCheckBox;
class QLabel;
class QLCDNumber;

class ControlGUI : public QMainWindow
{
    Q_OBJECT

public:
    ControlGUI( QWidget* parent = 0, const char* name = 0, WFlags fl = WType_TopLevel );
    ~ControlGUI();

    TribotsTools::FieldOfPlay* field;
    QGroupBox* groupBox1;
    QPushButton* pushButtonExecColorToolOmni;
    QPushButton* pushButtonExecColorToolPerspective;
    QPushButton* pushButtonExecCalibrationTool;
    QPushButton* pushButtonExecMarkerEditor;
    QPushButton* pushButtonExecCoriander;
    QPushButton* pushButtonExecConfigEditor;
    QPushButton* pushButtonExecJournalViewer;
    QPushButton* pushButtonExecTribotsview;
    QPushButton* pushButtonDebugImage;
    QGroupBox* groupBox2;
    QPushButton* pushButtonExecRobotcontrol;
    QPushButton* pushButtonExecRestartCaller;
    QPushButton* pushButtonExitRobotcontrol;
    QComboBox* comboBoxPlayertype;
    QCheckBox* checkBoxBlackScreen;
    QComboBox* comboBoxRefereeState;
    QComboBox* comboBoxPlayerrole;
    QPushButton* pushButtonActivate;
    QPushButton* pushButtonDeactivate;
    TribotsTools::MonoLED* monoLEDActivated;
    QLabel* textLabel2;
    TribotsTools::MonoLED* monoLEDConnection;
    QLabel* textLabel3;
    QLabel* textLabel1;
    QLCDNumber* lCDNumberVoltage;
    QToolBar *ToolbarMouseAction;
    QAction* SLHintAction;
    QAction* QuitAction;
    QAction* GotoAction;

public slots:
    virtual void init();
    virtual void destroy();
    virtual void playerType( const QString & s );
    virtual void playerRole( const QString & s );
    virtual void refereeState( const QString & s );
    virtual void slHintClicked();
    virtual void slHintActivated();
    virtual void keyPressEvent( QKeyEvent * ev );
    virtual void robotGoTo(Tribots::Vec,Tribots::Angle);
    virtual void gotoActivated();

protected:
    QGridLayout* ControlGUILayout;
    QVBoxLayout* layout7;
    QSpacerItem* spacer3;
    QGridLayout* groupBox1Layout;
    QGridLayout* groupBox2Layout;
    QGridLayout* layout12;
    QHBoxLayout* layout11;
    QSpacerItem* spacer4;
    QSpacerItem* spacer5;

protected slots:
    virtual void languageChange();

private:
    QTimer timer;
    DeactivateWidget* extraDeactivateButton;
    TribotsTools::ControlGUIState state;
    Tribots::TribotsUDPCommunication comm;
    Tribots::Time activationTime;
    bool activateRequest;
    int msecUntilActivation;
    int blackScreenMode;
    int latestBlackScreenMode;
    QTextEdit* journalWindow;
    Tribots::Time timeDebugImage;
    int robotcontrol_child_pid;
    std::string filepath;

    QPixmap image0;
    QPixmap image1;
    QPixmap image2;
    QPixmap image3;

private slots:
    virtual void activate();
    virtual void deactivate();
    virtual void debugImage();
    virtual void cycleTask();
    virtual void send();
    virtual void receive();
    virtual void updateDisplay();
    virtual void execRobotcontrol();
    virtual void quitRobotcontrol();
    virtual void execRestartCaller();
    virtual void execColorToolOmni();
    virtual void execColorToolPerspective();
    virtual void execCalibrationTool();
    virtual void execMarkerEditor();
    virtual void execCoriander();
    virtual void execConfigEditor();
    virtual void execJournalViewer();
    virtual void execTribotsview();
    virtual void toggleBlackScreen(bool);
    virtual void killChild();

};

#endif // CONTROLGUI_H
