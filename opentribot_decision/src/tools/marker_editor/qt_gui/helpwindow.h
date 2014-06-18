#ifndef HELPWINDOW_H
#define HELPWINDOW_H

#include <qapplication.h>

#include <qvbox.h>
#include <qhbox.h>
#include <qlabel.h>
#include <qpushbutton.h>
#include <qtextedit.h>
#include <qfont.h>

class helpWindow : public QWidget
{
        Q_OBJECT
    public:
        helpWindow( QWidget *parent=0, const char *name=0 );
        ~helpWindow() {}


    public slots:

    signals:


    private:
        QVBox *baselayout;
        QTextEdit *helptext;
        QPushButton *closeButton;
};
#endif // HELPWINDOW_H
