
#include "ScrollImageWidget.h"
#include <QtGui/QScrollBar>
#include <QtGui/QAbstractScrollArea>


using namespace TribotsTools;

ScrollImageWidget::ScrollImageWidget(QWidget* parent, Qt::WFlags f) : QScrollArea (parent), imgWidget(NULL) {
  imgWidget = new ImageWidget (NULL, f);
  init ();
}

ScrollImageWidget::ScrollImageWidget(const Tribots::Image& image, QWidget* parent, Qt::WFlags f) : QScrollArea (parent), imgWidget(NULL) {
  imgWidget = new ImageWidget (image, NULL, f);
  init ();
}

void ScrollImageWidget::init () {
  connect (imgWidget,SIGNAL(mousePressed(QMouseEvent*)),this,SLOT(mousePressEvent(QMouseEvent*)));
  connect (imgWidget,SIGNAL(mouseMoved(QMouseEvent*)),this,SLOT(mouseMoveEvent(QMouseEvent*)));
  connect (imgWidget,SIGNAL(keyPressed(QKeyEvent*)),this,SLOT(keyPressEvent(QKeyEvent*)));
  setHorizontalScrollBarPolicy (Qt::ScrollBarAsNeeded);
  setVerticalScrollBarPolicy (Qt::ScrollBarAsNeeded);
  setWidget(imgWidget);
}

ScrollImageWidget::~ScrollImageWidget() {
  if (imgWidget)
    delete imgWidget;
}

void ScrollImageWidget::setImage(const Image& new_image) {
  imgWidget->setImage(new_image);
}
void ScrollImageWidget::setImage(const ImageBuffer& new_image) {
  imgWidget->setImage(new_image);
}

void ScrollImageWidget::mousePressEvent(QMouseEvent* ev) {
  emit (mousePressed(ev));
}
void ScrollImageWidget::mouseMoveEvent(QMouseEvent* ev) {
  emit (mouseMoved(ev));
}
void ScrollImageWidget::keyPressEvent(QKeyEvent* ev) {
  emit (keyPressed(ev));
}

void ScrollImageWidget::centerImage () {
  if (imgWidget->height() < height())
    verticalScrollBar()->setValue (0);
  else
    verticalScrollBar()->setValue (verticalScrollBar()->maximum()/2);
  if (imgWidget->width() < width())
    horizontalScrollBar()->setValue (0);
  else
    horizontalScrollBar()->setValue (horizontalScrollBar()->maximum()/2);
}
