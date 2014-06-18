/****************************************************************************
** Form interface generated from reading ui file 'DistanceCalibrationCollectionWidget.ui'
**
** Created: Tue Dec 15 15:47:26 2009
**
** WARNING! All changes made in this file will be lost!
****************************************************************************/

#ifndef DISTANCECALIBRATIONCOLLECTIONWIDGET_H
#define DISTANCECALIBRATIONCOLLECTIONWIDGET_H

#include <qvariant.h>
#include <qpixmap.h>
#include <qmainwindow.h>
#include <vector>
#include <string>
#include "../../Fundamental/ConfigReader.h"
#include "DistanceCalibrationImageAnalysis.h"
#include "RGBColorClassifier.h"
#include "DistanceCalibrationAux.h"
#include "../../ImageProcessing/Formation/ImageSource.h"
#include "../../ImageProcessing/Formation/YUVImage.h"
#include "../../ImageProcessing/Formation/RGBImage.h"
#include "../../Robot/Robot.h"
#include "ImageMaskBuilder.h"
#include "../../WorldModel/WorldModel.h"
#include "../../ImageProcessing/Calibration/centerRingOperation.h"
#include "DistMarkerBuilder.h"

class QVBoxLayout;
class QHBoxLayout;
class QGridLayout;
class QSpacerItem;
class QAction;
class QActionGroup;
class QToolBar;
class QPopupMenu;
namespace TribotsTools {
class ScrollImageWidget;
}
class QPushButton;
class QGroupBox;
class QCheckBox;
class QButtonGroup;
class QRadioButton;
class QLabel;
class QSlider;

class DistanceCalibrationCollectionWidget : public QMainWindow
{
    Q_OBJECT

public:
    DistanceCalibrationCollectionWidget( QWidget* parent = 0, const char* name = 0, WFlags fl = WType_TopLevel );
    ~DistanceCalibrationCollectionWidget();

    TribotsTools::ScrollImageWidget* imageWidget;
    QPushButton* pushButtonAutoCenter;
    QPushButton* pushButtonResetRed;
    QPushButton* pushButtonBuildMarker;
    QPushButton* pushButtonBuildMask;
    QPushButton* pushButtonStop;
    QGroupBox* groupBox1;
    QCheckBox* checkBoxShowMask;
    QCheckBox* checkBoxShowSegmentation;
    QCheckBox* checkBoxShowTransitions;
    QCheckBox* checkBoxShowBalance;
    QCheckBox* checkBoxShowMarker;
    QCheckBox* checkBoxShowFreeze;
    QCheckBox* checkBoxShowDirection;
    QCheckBox* checkBoxShowCenter;
    QButtonGroup* buttonGroup1;
    QRadioButton* radioButtonSetCenter;
    QRadioButton* radioButtonSetDirection;
    QRadioButton* radioButtonSetBalance;
    QRadioButton* radioButtonSetRed;
    QRadioButton* radioButtonSetMaskAdd;
    QRadioButton* radioButtonSetMaskSub;
    QGroupBox* groupBox2;
    QLabel* textLabel3;
    QLabel* textLabel1;
    QSlider* sliderMaskThreshold;
    QSlider* sliderMaskDilation;
    QSlider* sliderCalibrationThreshold;
    QMenuBar *MenuBarEditor;
    QPopupMenu *Datei;
    QPopupMenu *popupMenu;
    QPopupMenu *popupMenu_5;
    QPopupMenu *popupMenu_7;
    QPopupMenu *popupMenu_10;
    QAction* openMaskAction;
    QAction* saveMaskAction;
    QAction* saveMaskAsAction;
    QAction* saveMarkerAction;
    QAction* saveMarkerAsAction;
    QAction* saveCenterAction;
    QAction* exitAction;
    QAction* saveExitAction;
    QAction* dateiMaskeAction;
    QAction* dateiMittelpunkt_und_BalancebereichAction;
    QAction* dateiDistanzmarkerAction;
    QAction* dateiDistanzlinien;
    QAction* saveDistlinesAction;
    QAction* saveDistlinesAsAction;

public slots:
    virtual void init();
    virtual void destroy();
    virtual void loop();

protected:
    Tribots::Time lastClick;
    std::string configfile;
    std::string imageSourceSection;
    TribotsTools::DCAutoInfo autoinfo;
    std::vector<std::vector<Tribots::RGBTuple> > colorSample;
    TribotsTools::DistanceCalibrationImageAnalysis imageAnalysis;
    Tribots::ImageSource* imageSource;
    Tribots::ConfigReader cfg;
    TribotsTools::RGBColorClassifier classifier;
    Tribots::RobotMask* robotMask;
    Tribots::Image* lastImage;
    TribotsTools::DCMode mode;
    Tribots::Time modeStarttime;
    Tribots::Time imageTimestamp;
    TribotsTools::DCMouseMode mouseMode;
    bool classifierChanged;
    int recentMouseClickX;
    int recentMouseClickY;
    Tribots::Robot* robot;
    std::string robotMaskFile;
    std::vector<TribotsTools::MarkerLog> markers;
    TribotsTools::ImageMaskBuilder imageMaskBuilder;
    bool debug;
    Tribots::WorldModel* wm;
    Tribots::RobotMask* robotMask2;
    TribotsTools::DistMarkerBuilder* distMarkerBuilder;
    bool distMarkerBuilderUsed;
    bool updateDirection;

    virtual void updateMaskManually( bool doAdd, int x, int y );
    virtual void saveMask( const std::string & filename );
    virtual void saveMarker( const std::string & filename );
    virtual void switchAutoFeatures( bool on );
    virtual void saveDistlines( const std::string & filename );

    QGridLayout* DistanceCalibrationCollectionWidgetLayout;
    QVBoxLayout* layout1;
    QGridLayout* groupBox1Layout;
    QGridLayout* buttonGroup1Layout;
    QGridLayout* groupBox2Layout;

protected slots:
    virtual void languageChange();

    virtual void setCenterActionClicked();
    virtual void setMaskAddActionClicked();
    virtual void setBlueActionClicked();
    virtual void setMaskSubActionClicked();
    virtual void setRedActionClicked();
    virtual void setDirectionActionClicked();
    virtual void setBalanceActionClicked();
    virtual void generateImageMaskStart();
    virtual void generateMarkerLogStart();
    virtual void stopActions();
    virtual void mouseInImagePressed( QMouseEvent * ev );
    virtual void mouseInImageMoved( QMouseEvent * ev );
    virtual void openMaskClicked();
    virtual void saveMaskClicked();
    virtual void saveMaskAsClicked();
    virtual void saveMarkerClicked();
    virtual void saveMarkerAsClicked();
    virtual void resetRedClicked();
    virtual void saveCenterClicked();
    virtual void recalculateMask( int );
    virtual void saveExitActionClicked();
    virtual void commandLineHelp();
    virtual void dilateMask( int );
    virtual void generateCenterBalanceArea();
    virtual void saveDistlinesClicked();
    virtual void saveDistlinesAsClicked();


private:
    int balanceY2;
    int balanceX2;
    int balanceY1;
    int balanceX1;

    QPixmap image0;

};

#endif // DISTANCECALIBRATIONCOLLECTIONWIDGET_H
