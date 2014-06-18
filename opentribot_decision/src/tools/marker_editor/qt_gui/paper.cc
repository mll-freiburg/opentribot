#include "paper.h"
#include <qpainter.h>
#include <qwmatrix.h>

#include <sstream>
#include <iostream>
#include <math.h>
#include <algorithm>

Paper::Paper( QWidget *parent, const char *name) : QWidget( parent, name )
{
   setPalette( Paper::COLOR_BACK() );
   setFixedSize( getWidth(), getHeight() );

   featList = new std::vector<feature>();
   markedFeatList = new std::vector<int>();

   distLines = 0;
   distLines = new std::vector<std::vector<feature> >();
   carpetParam = 0;
   carpetParam = new std::vector<double> ();

/*
   std::vector<feature> testline;
   feature f;
   f.angle = 10;
   f.dist = 12;
   f.type = "undefined";
   testline.push_back(f);
   f.angle = 30;
   f.dist = 12;
   testline.push_back(f);
   f.angle = 90;
   f.dist = 22;
   testline.push_back(f);
   f.angle = 180;
   f.dist = 2;
   testline.push_back(f);
   
   if (distLines != 0)
      distLines->push_back(testline);
*/

   bw_selectable = true;
   wb_selectable = true;
   wr_selectable = true;
   rm_selectable = true;
   rw_selectable = true;
}


int Paper::getWidth() {
    return 2*margin + (int)(logicalWidth*zoom)+leftextra;
}


int Paper::getHeight() {
    return 2*margin + (int)(logicalHeight*zoom)+10;
}


void Paper::addFeature(double angle, double dist, std::string type) {
    feature f;
    f.angle = angle;
    f.dist = dist;
    f.type = type;

    featList->push_back(f);
}


void Paper::setLines(std::vector<std::vector<feature> > *distLinesToSet, std::vector<double> *carpetParam) {
   if (this->distLines != 0)
      delete this->distLines;
   this->distLines = distLinesToSet;
   if (this->carpetParam != 0)
      delete this->carpetParam;
   this->carpetParam = carpetParam;
}


void Paper::initFeatures() {
    featList->clear();
    markedFeatList->clear();
}


void Paper::toogleBW() {
    bw_selectable = !bw_selectable;
}


void Paper::toogleWB() {
    wb_selectable = !wb_selectable;
}


void Paper::toogleWR() {
    wr_selectable = !wr_selectable;
}


void Paper::toogleRM() {
    rm_selectable = !rm_selectable;
}


void Paper::toogleRW() {
    rw_selectable = !rw_selectable;
}


bool Paper::isTypeSelectable(std::string type) {
    if (type == "bw")
        return bw_selectable;
    if (type == "wb")
        return wb_selectable;
    if (type == "wr")
        return wr_selectable;
    if (type == "rm")
        return rm_selectable;
    if (type == "rw")
        return rw_selectable;

    return true;
}


void Paper::paintEvent( QPaintEvent * )
{
    //std::cout << "method paintEvent\n"; 
    QPainter paint( this );

    // draw coordinate-system
    //
    // x
    paint.drawLine( margin+leftextra, getHeight() - margin, 
                    getWidth() - margin +leftextra, getHeight() - margin);
    QString xLabel = "";
    int xpos;
    for (int i=0; i <= logicalWidth; i += 60) {
        if ( 0!=i) {
            xpos = margin + (int)(i*zoom) + leftextra;
            xLabel = QString::number(i);
            paint.drawText( xpos-25, getHeight() - margin - 5, xLabel ); // number as label
            //paint.drawLine( xpos, getHeight() - margin +5, 
            //                xpos, getHeight() - margin -5); // vertical slice
            // vertical background bar
            paint.setPen( QColor(200,200,200));
            paint.drawLine( xpos, getHeight(), xpos, 0);
            paint.setPen( QColor(0,0,0));
            
        }
    }

   //
   // y
   paint.drawLine( margin+leftextra, margin, 
                   margin+leftextra, getHeight() - margin);
   QString yLabel = "";
   for (int i=0; i <= logicalHeight; i += 50) {
      if ( 0!=i) {
         yLabel = QString::number(i);
         paint.drawText( 0, getHeight() - margin - (int)(i*zoom) + 3, 
                         yLabel );
         paint.drawLine( margin+leftextra-5, 
                         getHeight() - margin - (int)(i*zoom), 
                         margin+leftextra+5, 
                         getHeight() - margin - (int)(i*zoom));
      }
   }

    // Plot data
    for (unsigned int i = 0; i < featList->size(); i++) {
        this->drawMarker(paint, ((*featList)[i]), false);
    }
   if ( 0 != markedFeatList->size() ) {
      for (int i=markedFeatList->size()-1; i>=0; i--) {
         drawMarker(paint,((*featList)[markedFeatList->at(i)]), true);
      }
   }

   // draw lines
   for (unsigned int i = 0; i < distLines->size(); i++) {
      //std::cout << "drawing line " << i 
      //          << " with real distance " << (*carpetParam)[i] << "\n";
      this->drawDistline(paint, (*distLines)[i], (*carpetParam)[i] );
    }
}


void Paper::drawDistline(QPainter &paint, std::vector<feature> &line,
                         double realDist) {
   if (line.size() == 0)
      return;
   QString lineLabel = QString::number(realDist);
   paint.drawText( getXDrawCoord(line[line.size()-1].angle)-20, 
                   getYDrawCoord(line[line.size()-1].dist), 
                   lineLabel);
   double min_angle = line[0].angle;
   double max_angle = line[0].angle;
   for (unsigned int i = 0; i < line.size(); i++) {
      max_angle = (line[i].angle > max_angle) ? line[i].angle : max_angle;
      min_angle = (line[i].angle < min_angle) ? line[i].angle : min_angle;
      this->drawCircle(paint,
                        getXDrawCoord(line[i].angle), 
                        getYDrawCoord(line[i].dist), 
                        linecircle);
      if (i+1<line.size()) // last line needs special treatment
         paint.drawLine( getXDrawCoord(line[i].angle), 
                         getYDrawCoord(line[i].dist), 
                         getXDrawCoord(line[i+1].angle), 
                         getYDrawCoord(line[i+1].dist) );
    }
    // calc intersection height 
    // (assuming the last point has highest angle and the first the lowest
    // otherwise the connection is not drawn)
    if ( (min_angle == line[0].angle) && 
         (max_angle == line[line.size()-1].angle) ) {
      double px = line[line.size()-1].angle;
      double py = line[line.size()-1].dist;
      double qx = line[0].angle;
      double qy = line[0].dist;
      double a = 360.0 - px;
      double height = py-((py-qy)*a)/(a+qx); // Strahlensatz
      
      paint.drawLine( getXDrawCoord(line[line.size()-1].angle), 
                      getYDrawCoord(line[line.size()-1].dist), 
                      getXDrawCoord(360), getYDrawCoord(height) );
      paint.drawLine( getXDrawCoord(0), getYDrawCoord(height), 
                      getXDrawCoord(line[0].angle), 
                      getYDrawCoord(line[0].dist) );
   }
}


void Paper::drawMarker(QPainter &paint, feature &f, bool highlight) {
   QColor color;   
   if (highlight) {
      color = QColor(255,0,0);
   } else {
      color = getTypeColor(f.type);
   }
   int x_coord = getXDrawCoord(f.angle);
   int y_coord = getYDrawCoord(f.dist);
    // draw +
    const int half_plus_width = 1;
    paint.setPen(color);
    paint.drawLine( x_coord-half_plus_width, y_coord, x_coord+half_plus_width, y_coord);
    paint.drawLine( x_coord, y_coord-half_plus_width, x_coord, y_coord+half_plus_width);
    paint.setPen(QColor(0,0,0));
}


void Paper::drawCircle(QPainter &paint, int x, int y, int radius) {
   int x_coord = x-radius;
   int y_coord = y-radius;
   paint.drawEllipse( x_coord, y_coord, 2*radius, 2*radius); // hopefully a circle
}


QColor Paper::getTypeColor(std::string type) {
   QColor color(0,0,0);
   if (type == "bw")
      color = Paper::COLOR_BW();
   if (type == "wb")
      color = Paper::COLOR_WB();
   if (type == "wr")
      color = Paper::COLOR_WR();
   if (type == "rm")
      color = Paper::COLOR_RM();
   if (type == "rw")
      color = Paper::COLOR_RW();
      
   return color;
}


bool Paper::getLinePoint(const QPoint &pos, int &line, int &point) {
   if ( (distLines == 0) || (distLines->size() == 0) )
      return false;
   // search next point
   for (unsigned int i=0; i<distLines->size(); i++) {
      if ( (*distLines)[i].size() != 0 ) {
         for (unsigned int j=0; j<(*distLines)[i].size(); j++) {
            if ( (fabs(getXDrawCoord((*distLines)[i][j].angle)-pos.x()) < linecircle) &&
                 (fabs(getYDrawCoord((*distLines)[i][j].dist)-pos.y()) < linecircle) ) {
                 line = i;
                 point = j;
                 return true;
            }
         }
      }
   }
   return false;
}


int Paper::getXDrawCoord(double origX) {
    return ((int) (origX*zoom) + margin + leftextra);
}


int Paper::getYDrawCoord(double origY) {
    return (getHeight() - (int)(origY*zoom) - margin);
}


double Paper::getXOrigCoord(int drawX) {
   return (drawX-margin-leftextra)/zoom;
}


double Paper::getYOrigCoord(int drawY) {
   return (getHeight() -drawY -margin)/zoom;
}


QSizePolicy Paper::sizePolicy() const
{
   return QSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );
}


void Paper::markFeatures() {
    // transform coordinates backwards
    QPoint transformedStart, transformedEnd;
    transformedStart.setX( (int)getXOrigCoord(mouseStart.x()) );
    transformedStart.setY( (int)getYOrigCoord(mouseStart.y()) );
    transformedEnd.setX( (int)getXOrigCoord(mouseEnd.x()) );
    transformedEnd.setY( (int)getYOrigCoord(mouseEnd.y()) );

    // get bounds
    int minX, minY, maxX, maxY;
    if (transformedStart.x() <= transformedEnd.x()) {
        minX = transformedStart.x();
        maxX = transformedEnd.x();
    } else {
        maxX = transformedStart.x();
        minX = transformedEnd.x();
    }
    if (transformedStart.y() <= transformedEnd.y()) {
        minY = transformedStart.y();
        maxY = transformedEnd.y();
    } else {
        maxY = transformedStart.y();
        minY = transformedEnd.y();
    }

    markedFeatList->clear();
    //std::cout << "searching angle: " << searchAngle << " and dist: " << searchDist << "\n";
    for (unsigned int i=0; i<featList->size(); i++) {
        if ( (featList->at(i).angle >= minX) && (featList->at(i).angle <= maxX) &&
             (featList->at(i).dist >= minY)  && (featList->at(i).dist <= maxY) &&
             isTypeSelectable(featList->at(i).type))
        {
            markedFeatList->push_back(i);
            //std::cout << "found point at (" << featList->at(i).angle << "; " << featList->at(i).dist << ")\n";
        }
    }
}


void Paper::mousePressEvent( QMouseEvent *e )
{
   if ( e->button() == QMouseEvent::LeftButton ) {
      QPainter paint( this );
      mouseStart = e->pos();
      mouseEnd = e->pos();
      btnPressed = true;
      int line, point;
      if ( getLinePoint(e->pos(), line, point) ) {
         // std::cout << "clicked at point " << point << " in line " << line << "\n";
         currentmod = move;
         currentline = line;
         currentpoint = point;
         // delete original circle
         paint.setRasterOp(Qt::XorROP);
         paint.setPen(Qt::black);
         this->drawCircle(paint,getXDrawCoord((*distLines)[currentline][currentpoint].angle), mouseEnd.y(), linecircle);
      } else {
         // start to paint an selection rect
         // if no distline-point is near the mouse 
         currentmod = mark;
         if ( 0 != markedFeatList->size() ) { // unmark previously marked markers
            for (int i=markedFeatList->size()-1; i>=0; i--) {
               drawMarker(paint,((*featList)[markedFeatList->at(i)]), false);
            }
         }
         markedFeatList->clear();
      }
   }

   // delete marked markers if right mb pressed
   if ( e->button() == QMouseEvent::RightButton)
   {
      deleteMarkedMarkers();
   }
}


void Paper::mouseReleaseEvent( QMouseEvent *e )
{
   if ( e->button() == QMouseEvent::LeftButton ) {
      btnPressed = false;
      QPainter paint( this );
      if ( currentmod == mark) {
         paint.drawWinFocusRect(QRect(mouseStart, mouseEnd)); // erase focus rect
         markFeatures(); // mark new marked features 
         if ( 0 != markedFeatList->size() ) {
            for (int i=markedFeatList->size()-1; i>=0; i--) {
               drawMarker(paint,((*featList)[markedFeatList->at(i)]), true);
            }
         }
      }
      if ( currentmod == move) {
         (*distLines)[currentline][currentpoint].dist=getYOrigCoord(mouseEnd.y());
         repaint();
      }
   }
}


void Paper::mouseMoveEvent( QMouseEvent *e )
{
   if (btnPressed) {
      if ( currentmod == mark) {
         QPainter paint( this );
         paint.drawWinFocusRect(QRect(mouseStart, mouseEnd));
         mouseEnd = e->pos();
         paint.drawWinFocusRect(QRect(mouseStart, mouseEnd));
      }
      if ( currentmod == move) {
         QPainter paint( this );
         paint.setRasterOp(Qt::NotXorROP);
         paint.setPen(Qt::black);
         //erase old
         this->drawCircle(paint,getXDrawCoord((*distLines)[currentline][currentpoint].angle), mouseEnd.y(), linecircle);
         mouseEnd = e->pos();
         // draw new xor circle
         this->drawCircle(paint,getXDrawCoord((*distLines)[currentline][currentpoint].angle), mouseEnd.y(), linecircle);
         paint.setRasterOp(Qt::CopyROP);
      }
    }
}

void Paper::deleteMarkedMarkers() {
   if (0 != markedFeatList->size()) {
      //delete backwards, as the index changes after each deletion
      sort(markedFeatList->begin(), markedFeatList->end());
      for (int i=markedFeatList->size()-1; i>=0; i--) {
         featList->erase(featList->begin() + markedFeatList->at(i));
      }
      markedFeatList->clear();
      repaint();
   } else {
   	std::cout << "deleting nothing as nothing was selected!\n";
   }
};
