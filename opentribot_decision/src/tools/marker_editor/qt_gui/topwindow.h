#ifndef TOPWINDOW_H
#define TOPWINDOW_H

#include <fstream>

#include <qapplication.h>
#include <qvbox.h>
#include <qlabel.h>
#include <qfont.h>
#include <qcheckbox.h>
#include <qmenubar.h>
#include "paper.h"
#include "helpwindow.h"
#include "selwindow.h"
#include "../../CalibrationTool/DistMarkerBuilder.h"

class topWindow : public QWidget
{
        Q_OBJECT
    public:
        topWindow( QWidget *parent=0, const char *name=0 );
        ~topWindow();


    public slots:
        void open();
        void save_marker();
        void save_lines();
        void save_markerDialog();
        void save_lineDialog();
        void generate_lines();
        void showHelp();
        void showSelection();
        void about();
        void terminateApplication();
        void saveAndExit();


    signals:


    private:
        /**
         * autoload loads marker.log from the given path and dist_marker.cfg 
         * from ~/.robotcontrol/
         */
        void autoload(std::string path);
        void createMENU();
        void connectSignalsToSlots();
        void loadMarkerlog(std::ifstream &src);
        void loadDistcfg(std::istream &src);
        bool isMarkerfileLoaded();

        struct distmarkerFileheader {
            int imagewidth;
            int imageheight;
            int center_x;
            int center_y;
            int exponent;
            unsigned int linecount;
        };
        distmarkerFileheader loadedDistFile;
        QString distMarkerFileName;

        QString markerFileName;

        QVBox *baselayout;
        QMenuBar *menu;
        Paper *paper;

        selWindow *selWnd;
        helpWindow *helpWnd;

        TribotsTools::DistMarkerBuilder *distMarkerBuilder;
};
#endif // TOPWINDOW_H
