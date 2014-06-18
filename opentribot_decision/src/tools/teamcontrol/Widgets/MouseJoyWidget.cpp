
#include "MouseJoyWidget.h"
#include "../States/RemoteBlackboard.h"
#include <qpainter.h>
#include <cmath>
#include <iostream>

using namespace std;

TribotsTools::MouseJoyWidget::MouseJoyWidget(QWidget* parent, const char * name, WFlags f)
  : QFrame(parent, name, f)
{
  setFocusPolicy(QWidget::StrongFocus);
  is_active=has_focus=button_pressed=mouse_inside=false;
  vx=vy=vphi=0;
  kick=false;
  setNoVel ();
}

void TribotsTools::MouseJoyWidget::mousePressEvent (QMouseEvent* ev) {
  debugState ("MPE");
  if (ev->x()>=0 && ev->y()>=0 && ev->x()<width() && ev->y()<height())
    mouse_inside=true;
  if (is_active && has_focus && mouse_inside && ev->button()==Qt::LeftButton) {
    button_pressed=true;
    double cx, cy, cr;
    centerRadius (cx, cy, cr);
    double dx = (ev->x()-cx)/cr;
    double dy = (ev->y()-cy)/cr;
    if (dx*dx+dy*dy>1) {
      double f = sqrt(dx*dx+dy*dy);
      dx/=f;
      dy/=f;
    }
    vx=dx;
    vy=dy;
    setJoyPos ();
  }
}

void TribotsTools::MouseJoyWidget::mouseReleaseEvent (QMouseEvent* ev) {
  debugState ("MRE");
  if (ev->button()==Qt::LeftButton)
    button_pressed=false;
  vx=vy=0;
  if (is_active && has_focus)
    setJoyPos();
}

void TribotsTools::MouseJoyWidget::mouseMoveEvent (QMouseEvent* ev) {
  debugState ("MME");
  if (is_active && has_focus && mouse_inside && button_pressed) {
    double cx, cy, cr;
    centerRadius (cx, cy, cr);
    double dx = (ev->x()-cx)/cr;
    double dy = (ev->y()-cy)/cr;
    if (dx*dx+dy*dy>1) {
      double f = sqrt(dx*dx+dy*dy);
      dx/=f;
      dy/=f;
    }
    vx=dx;
    vy=-dy;
    setJoyPos ();
  }
}

void TribotsTools::MouseJoyWidget::enterEvent (QEvent*) {
  debugState ("EE");
  mouse_inside=true;
  if (has_focus && is_active) {
    setJoyPos();
  }
}

void TribotsTools::MouseJoyWidget::leaveEvent (QEvent*) {
  debugState ("LE");
  mouse_inside=false;
  setNoVel();
}

void TribotsTools::MouseJoyWidget::focusInEvent (QFocusEvent*) {
  debugState ("FIE");
  has_focus=true;
}

void TribotsTools::MouseJoyWidget::focusOutEvent (QFocusEvent*) {
  debugState ("FOE");
  has_focus=false;
  vx=vy=vphi=0;
  kick=false;
  setJoyPos();  
}

void TribotsTools::MouseJoyWidget::activate (bool b) {
  is_active=b;
  button_pressed=false;
  if (is_active==false) {
    vx=vy=vphi=0;
    kick=false;
    setJoyPos();
  }
}

void TribotsTools::MouseJoyWidget::keyPressEvent (QKeyEvent* ev) {
  debugState ("KPE");
  if (is_active && has_focus && mouse_inside) {
    if (ev->key()==Qt::Key_A) {
      vphi=1;
    } else if (ev->key()==Qt::Key_S) {
      vphi=-1;
    } else if (ev->key()==Qt::Key_Shift) {
      kick=true;
    }
    setJoyPos();
  }
}
  
void TribotsTools::MouseJoyWidget::keyReleaseEvent (QKeyEvent* ev) {
  debugState ("KRE");
  if (ev->key()==Qt::Key_Shift)
    kick=false;
  else if (ev->key()==Qt::Key_A || ev->key()==Qt::Key_S)
    vphi=0;
  if (is_active && has_focus && mouse_inside)
    setJoyPos();
}

void TribotsTools::MouseJoyWidget::paintEvent(QPaintEvent *) {
  QPainter p(this);
  p.setPen(QPen(QColor(0,0,0), 2));
  double cx, cy, cr;
  centerRadius (cx,cy,cr);
  p.drawArc (static_cast<int>(cx-cr), static_cast<int>(cy-cr),static_cast<int>(2*cr), static_cast<int>(2*cr),0,5760);
  p.drawArc (static_cast<int>(cx-2), static_cast<int>(cy-2),static_cast<int>(4), static_cast<int>(4),0,5760);
  if (is_active && has_focus && button_pressed) {
    p.setPen(Qt::red);
    p.drawArc (static_cast<int>(cx+cr*vx-2), static_cast<int>(cy-cr*vy-2),static_cast<int>(4), static_cast<int>(4),0,5760);
  }
}

void TribotsTools::MouseJoyWidget::centerRadius (double& cx, double& cy, double& cr) {
  double w = width();
  double h = height ();
  cr = 0.4*(w<h ? w : h);
  cx=0.5*w;
  cy=0.5*h;
}

void TribotsTools::MouseJoyWidget::setJoyPos () {
  REMBB.joystick_state.vx=vx*REMBB.joystick_state.max_vx;
  REMBB.joystick_state.vy=vy*REMBB.joystick_state.max_vy;
  REMBB.joystick_state.kick=kick;
  REMBB.joystick_state.vphi=vphi*REMBB.joystick_state.max_vphi;
  update();
}

void TribotsTools::MouseJoyWidget::setNoVel () {
  REMBB.joystick_state.vx=0;
  REMBB.joystick_state.vy=0;
  REMBB.joystick_state.kick=false;
  REMBB.joystick_state.vphi=0;
  update();
}

void TribotsTools::MouseJoyWidget::debugState (const char* /*m*/) {
/*  cerr << m << ": "
      << (is_active ? "act" : "inact") << " "
      << (has_focus ? "foc" : "nofoc") << " "
      << (mouse_inside ? "mouse" : "nomouse") << " "
  << (button_pressed ? "butt" : "nobutt") << endl;*/
}
