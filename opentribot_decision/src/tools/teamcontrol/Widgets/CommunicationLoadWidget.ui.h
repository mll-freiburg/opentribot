/****************************************************************************
** ui.h extension file, included from the uic-generated form implementation.
**
** If you want to add, delete, or rename functions or slots, use
** Qt Designer to update this file, preserving your code.
**
** You should not define a constructor or destructor in this file.
** Instead, write your code in functions called init() and destroy().
** These will automatically be called by the form's constructor and
** destructor.
*****************************************************************************/


void CommunicationLoadWidget::init()
{
  latest_update.start();
  valPlotWidgetPackets->init_widget_multi (0.0, 70.0, 120.0, 2);
  valPlotWidgetBytes->init_widget_multi (0.0, 100.0, 120.0, 2);
}

void CommunicationLoadWidget::update()
{
  if (latest_update.elapsed()>=1000) {
    latest_update = latest_update.addMSecs (1000);
    double send_packets = 0;
    double send_bytes = 0;
    double receive_packets = 0;
    double receive_bytes = 0;
    for (unsigned int i=0; i<REMBB.robot_state.size(); i++) {
      double sp = REMBB.robot_state[i].comm_statistics_send.packet_rate;
      send_packets+=sp;
      send_bytes+=sp*REMBB.robot_state[i].comm_statistics_send.packet_size;
      double rp = REMBB.robot_state[i].comm_statistics_receive.packet_rate;
      receive_packets+=rp;
      receive_bytes+=rp*REMBB.robot_state[i].comm_statistics_receive.packet_size;
    }

    lineEditBytesSend->setText (QString::number (send_bytes/128, 'f', 1));
    lineEditBytesReceive->setText (QString::number (receive_bytes/128, 'f', 1));
    lineEditPacketsSend->setText (QString::number (send_packets, 'f', 1));
    lineEditPacketsReceive->setText (QString::number (receive_packets, 'f', 1));
    valPlotWidgetPackets->push (send_packets, 0);
    valPlotWidgetPackets->push (receive_packets, 1);
    valPlotWidgetPackets->repaint ();
    valPlotWidgetBytes->push (send_bytes/128, 0);  // waren urspruenglich als Bytes vorgesehen, jetzt kb/s
    valPlotWidgetBytes->push (receive_bytes/128, 1);
    valPlotWidgetBytes->repaint ();
  }
}
