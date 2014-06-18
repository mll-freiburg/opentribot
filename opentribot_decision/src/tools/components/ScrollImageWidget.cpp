
#include "ScrollImageWidget.h"

using namespace TribotsTools;

ScrollImageWidget::ScrollImageWidget( QWidget* parent, const char* name, WFlags flags) : QScrollView (parent, name, flags) {
  imgWidget = new ImageWidget (this->viewport());
  init ();
}

void ScrollImageWidget::init () {
  connect (imgWidget,SIGNAL(mousePressed(QMouseEvent*)),this,SLOT(mousePressEvent(QMouseEvent*)));
  connect (imgWidget,SIGNAL(mouseMoved(QMouseEvent*)),this,SLOT(mouseMoveEvent(QMouseEvent*)));
  connect (imgWidget,SIGNAL(keyPressed(QKeyEvent*)),this,SLOT(keyPressEvent(QKeyEvent*)));
}

ScrollImageWidget::~ScrollImageWidget() {
  if (imgWidget)
    delete imgWidget;
}

void ScrollImageWidget::setImage(const Image& new_image) {
  imgWidget->setImage(new_image);
  resizeContents (new_image.getWidth(),new_image.getHeight());
}
void ScrollImageWidget::setImage(const ImageBuffer& new_image) {
  imgWidget->setImage(new_image);
  resizeContents (new_image.width,new_image.height);
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
    verticalScrollBar()->setValue (verticalScrollBar()->maxValue()/2);
  if (imgWidget->width() < width())
    horizontalScrollBar()->setValue (0);
  else
    horizontalScrollBar()->setValue (horizontalScrollBar()->maxValue()/2);
}
