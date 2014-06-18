#ifndef SELWINDOW_H
#define SELWINDOW_H

#include <qapplication.h>
#include <qvbox.h>
#include <qhbox.h>
#include <qlabel.h>
#include <qpushbutton.h>

#include "paper.h"

class selWindow : public QWidget
{
        Q_OBJECT
    public:
        selWindow( Paper *paper, QWidget *parent=0, const char *name=0 );
        ~selWindow() {}


    public slots:

    signals:


    private:
        QVBox *baselayout;
        QPushButton *closeButton;

        Paper *paper;
};
#endif // SELWINDOW_H
