
#ifndef _TribotsTools_TribotsSyntax_h_
#define _TribotsTools_TribotsSyntax_h_

#include <QtGui/QSyntaxHighlighter>

namespace TribotsTools {

  class TribotsSyntax: public QSyntaxHighlighter {
  public:
     TribotsSyntax(QTextEdit *textEdit);
    virtual ~TribotsSyntax(void);

    virtual void highlightBlock(const QString &text);
  };

}

#endif
