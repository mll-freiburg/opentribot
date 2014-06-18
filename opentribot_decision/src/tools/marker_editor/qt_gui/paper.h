#ifndef PAPER_H
#define PAPER_H

#include <qwidget.h>
#include <qpoint.h>

#include <string>
#include <vector>

struct feature {
    double angle;
    double dist;
    std::string type;
};

class Paper : public QWidget
{
      Q_OBJECT
    public:
        static QColor COLOR_BACK() { return QColor(255,255,255); }
        static QColor COLOR_BW() { return QColor(0,128,255); }
        static QColor COLOR_WB() { return QColor(0,255,128); }
        static QColor COLOR_WR() { return QColor(255,255,0); }
        static QColor COLOR_RM() { return QColor(255,128,64); }
        static QColor COLOR_RW() { return QColor(128,0,0); }

        Paper( QWidget *parent=0, const char *name=0);
        ~Paper(){}
        int getWidth();
        int getHeight();
        QSizePolicy sizePolicy() const;
        std::vector<feature> * getFeatureList() { return featList; }
        std::vector<std::vector<feature> > * getLines() { return distLines; }
        std::vector<double> * getCarpetparam() { return carpetParam; }
        void addFeature(double angle, double dist, std::string type);
        void setLines(std::vector<std::vector<feature> > *distLinesToSet, 
                      std::vector<double> *carpetParam);
        void initFeatures();

    public slots:
        void toogleBW();
        void toogleWB();
        void toogleWR();
        void toogleRM();
        void toogleRW();
        void deleteMarkedMarkers();


    signals:

    protected:
        bool isTypeSelectable(std::string type);
        void paintEvent( QPaintEvent * );
        void markFeatures();
        void mousePressEvent( QMouseEvent *e );
        void mouseReleaseEvent( QMouseEvent *e );
        void mouseMoveEvent( QMouseEvent *e );

    private:
        void drawCircle(QPainter &paint, int x, int y, int radius);
        void drawMarker(QPainter &paint, feature &p, bool highlight);
        void drawDistline(QPainter &paint, 
                          std::vector<feature> &line,
                          double realDist);
        QColor getTypeColor(std::string type);
        bool getLinePoint(const QPoint &pos, int &line, int &point);

        int getXDrawCoord(double origX);
        int getYDrawCoord(double origY);
        double getXOrigCoord(int drawX);
        double getYOrigCoord(int drawY);

        static const int logicalWidth = 360;     //　degrees
        static const int logicalHeight = 260;    //  pixel from image source
        static const int margin = 5;            //  additional margin in pixel used for coordinate system
        static const int leftextra = 12;         //  additional left margin 
        static const int zoom = 2;
        static const int linecircle = 4;         // radius of distance lince circle
        
        enum mousemod { move, mark };

        mousemod currentmod;
        int currentline, currentpoint;

        std::vector<feature> *featList;
        std::vector<int> *markedFeatList;

        std::vector<std::vector<feature> > *distLines;
        std::vector<double> *carpetParam;

        bool btnPressed;                        // recognize left mouse button
        QPoint mouseStart, mouseEnd;

        bool bw_selectable;
        bool wb_selectable;
        bool wr_selectable;
        bool rm_selectable;
        bool rw_selectable;
};
#endif // PAPER_H

