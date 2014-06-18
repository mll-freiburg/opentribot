#ifndef _SAVINGOBJECT_H_
#define _SAVINGOBJECT_H_

#include <qwidget.h>

namespace TribotsTools {

  /** Widget, um Funktionsverlaeufe ueber der Zeit zu plotten */
  class SavingObject : public QObject
  {
  Q_OBJECT

    private:
      bool isSet;

    public slots:
      SavingObject () : isSet(false) {;}
      void acceptKey ( QKeyEvent* ev ) {
        if (ev->key()==Key_S)
              isSet=true;
      }
      bool check () { bool v=isSet; isSet=false; return v; }
  };

}

#endif
