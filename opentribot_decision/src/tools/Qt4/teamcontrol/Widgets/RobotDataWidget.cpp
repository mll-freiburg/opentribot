
#include "RobotDataWidget.h"
#include "../States/RemoteBlackboard.h"

using namespace TribotsTools;

RobotDataWidget::RobotDataWidget(QWidget* p, Qt::WindowFlags f) : QWidget (p,f)
{
  setupUi(this);

  ValPlotWidgetVcc->init_widget(35, 100);

  ValPlotWidgetT1->init_widget(200, 100);
  ValPlotWidgetT2->init_widget(200, 100);
  ValPlotWidgetT3->init_widget(200, 100);

  ValPlotWidgetC1->init_widget(5, 100);
  ValPlotWidgetC2->init_widget(5, 100);
  ValPlotWidgetC3->init_widget(5, 100);

  ValPlotWidgetO1->init_widget(-110, 110, 100);
  ValPlotWidgetO2->init_widget(-110, 110, 100);
  ValPlotWidgetO3->init_widget(-110, 110, 100);
}

RobotDataWidget::~RobotDataWidget ()
{
  // nichts zu tun
}

void RobotDataWidget::update()
{
  const Tribots::RobotData& rd (REMBB.robot_state[robot].robot_data);

  QPalette qpal_red;
  QPalette qpal_green;
  qpal_red.setBrush (QPalette::Background, Qt::red);
  qpal_green.setBrush (QPalette::Background, Qt::green);

  if (!rd.motor_temp_switch[0]) lCDNumberTemp1->setPalette(qpal_green);
  else lCDNumberTemp1->setPalette(qpal_red);
  if (!rd.motor_temp_switch[1]) lCDNumberTemp2->setPalette(qpal_green);
  else lCDNumberTemp2->setPalette(qpal_red);
  if (!rd.motor_temp_switch[2]) lCDNumberTemp3->setPalette(qpal_green);
  else lCDNumberTemp3->setPalette(qpal_red);

  lCDNumberVers->display(rd.BoardID);

  textLabelIDString->setText(QString(rd.robotIdString));

  lCDNumberVcc->display(rd.motor_vcc);
  ValPlotWidgetVcc->push(rd.motor_vcc);
  ValPlotWidgetVcc->repaint();

  lCDNumberTemp1->display(rd.motor_temp[0]);
  ValPlotWidgetT1->push(rd.motor_temp[0]);
  ValPlotWidgetT1->repaint();

  lCDNumberTemp2->display(rd.motor_temp[1]);
  ValPlotWidgetT2->push(rd.motor_temp[1]);
  ValPlotWidgetT2->repaint();

  lCDNumberTemp3->display(rd.motor_temp[2]);
  ValPlotWidgetT3->push(rd.motor_temp[2]);
  ValPlotWidgetT3->repaint();

  lCDNumberC1->display(rd.motor_current[0]);
  ValPlotWidgetC1->push(rd.motor_current[0]);
  ValPlotWidgetC1->repaint();

  lCDNumberC2->display(rd.motor_current[1]);
  ValPlotWidgetC2->push(rd.motor_current[1]);
  ValPlotWidgetC2->repaint();

  lCDNumberC3->display(rd.motor_current[2]);
  ValPlotWidgetC3->push(rd.motor_current[2]);
  ValPlotWidgetC3->repaint();

  lCDNumberO1->display(rd.motor_output[0]*100);
  ValPlotWidgetO1->push(rd.motor_output[0]*100);
  ValPlotWidgetO1->repaint();

  lCDNumberO2->display(rd.motor_output[1]*100);
  ValPlotWidgetO2->push(rd.motor_output[1]*100);
  ValPlotWidgetO2->repaint();

  lCDNumberO3->display(rd.motor_output[2]*100);
  ValPlotWidgetO3->push(rd.motor_output[2]*100);
  ValPlotWidgetO3->repaint();

  QWidget::update();
}

void RobotDataWidget::init( unsigned int n)
{
  robot=n;
  this->setWindowTitle((std::string("Robot Data ")+REMBB.robot_state[n].name).c_str());
}
