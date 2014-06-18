

#include <iostream>

#include <QtCore/qregexp.h>
#include <QtCore/qstring.h>
#include <QtCore/qstringlist.h>

#include "TribotsSyntax.h"

using namespace TribotsTools;

TribotsSyntax::TribotsSyntax(QTextEdit *textEdit)
  : QSyntaxHighlighter(textEdit) {
  return;
}

TribotsSyntax::~TribotsSyntax(void) {
  return;
}

void TribotsSyntax::highlightBlock(const QString &text) {
  int start=0;
  QStringList lines = text.split('\n');
  for(unsigned int i=0; static_cast<int>(i)<lines.count(); i++) {
//    int commandLength = 0;
    QString line = lines[i];
    int lineLength = line.length() + 1;

    if(line.isNull() || line.isEmpty()) {
//      std::cout << "is null or empty" << std::endl;
      setFormat(start, lineLength, QColor(255, 0, 0));
      start += lineLength;
      continue;
    }

    for(int i=0; i<lineLength-1; i++) {
      QChar a = line[i];

      if(a == '#') {
        setFormat(start+i, lineLength-i, QColor(150, 150, 150));
        break;
      }

      if(a == '=') {
        setFormat(start+i, lineLength-i, QColor(0, 0, 155));

      }

      if(a == '[') {
        setFormat(start+i, lineLength-i, QColor(155, 0, 0));

      }
      if(a == '+') {
        setFormat(start+i, lineLength-i, QColor(0,155, 0));

      }
    }
  }

}
