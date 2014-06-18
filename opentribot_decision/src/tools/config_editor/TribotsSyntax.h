
#include "qsyntaxhighlighter.h"

class TribotsSyntax: public QSyntaxHighlighter {
public:
   TribotsSyntax(QTextEdit *textEdit);
   virtual ~TribotsSyntax(void);

   int highlightParagraph(const QString &text, int endStateOfLastPara);
};
