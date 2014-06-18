/****************************************************************************
** ui.h extension file, included from the uic-generated form implementation.
**
** If you wish to add, delete or rename functions or slots use
** Qt Designer which will update this file, preserving your code. Create an
** init() function in place of a constructor, and a destroy() function in
** place of a destructor.
*****************************************************************************/


void RobotDataWidget::init()
{
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


void RobotDataWidget::update()
{
  const Tribots::RobotData& rd (REMBB.robot_state[robot].robot_data);
      
  if (!rd.motor_temp_switch[0]) lCDNumberTemp1->setPaletteBackgroundColor(QColor(0,255,0));
  else lCDNumberTemp1->setPaletteBackgroundColor(QColor(255,0,0));
  if (!rd.motor_temp_switch[1]) lCDNumberTemp2->setPaletteBackgroundColor(QColor(0,255,0));
  else lCDNumberTemp2->setPaletteBackgroundColor(QColor(255,0,0));
  if (!rd.motor_temp_switch[2]) lCDNumberTemp3->setPaletteBackgroundColor(QColor(0,255,0));
  else lCDNumberTemp3->setPaletteBackgroundColor(QColor(255,0,0));
  
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
  this->setCaption((std::string("Robot Data ")+REMBB.robot_state[n].name).c_str());
}



