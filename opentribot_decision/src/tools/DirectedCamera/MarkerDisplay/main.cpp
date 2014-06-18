
#include "MarkerWidget.h"
#include <qapplication.h>
#include <qtimer.h>
#include <iostream>
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <sstream>

using namespace std;
using namespace TribotsTools;

int main (int argc, char** argv) {
  try{
    if (argc==2 && ((string(argv[1])=="--help") || string(argv[1])=="-h")) {
      cerr << "Programm zum TCP-gesteuerten Anzeigen von Kalibrierpunkten auf dem Bildschirm\n";
      cerr << argv[0] << " [-f] [-h|--help] [-x XRES] [-y YRES]\n";
      cerr << "  -f startet das Programm im Full-Screen-Modus\n";
      cerr << "  -x XRES, -y YRES geben die horizontale bzw. vertikale  Aufloesung des Bildschirms\n";
      cerr << "  an wobei XRES bzw. YRES die Anzahl Pixel pro mm bedeuten. XRES, YRES koennen\n";
      cerr << "  Nachkommastellen besitzen\n";
      return 1;
    }
    bool fullScreen=false;
    double xres=-1;
    double yres=-1;
    for (int i=1; i<argc; i++) {
      string arg (argv[i]);
      if (arg=="-f") fullScreen=true;
      if (arg=="-x" && i+1<argc) xres=atof(argv[++i]);
      if (arg=="-y" && i+1<argc) yres=atof(argv[++i]);
    }
    QApplication a (argc, argv);
    MarkerWidget w;
    qApp->setMainWidget(&w);
    w.setPosition (500,300);
    QTimer timer;
    a.connect ( &timer, SIGNAL(timeout()), &w, SLOT(switchmode()));
    a.connect( &a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()) );
    while (xres<=0 || yres<=0) {
      w.setMode(MarkerWidget::blackScreen);
      w.show();
      w.repaint();
      a.processEvents();
      cout << "The Widget shows a red rectangle.\n";
      cout << "Measure its size and give the width and height in millimeters.\n";
      cout << "If cou cannot see a rectangle enter -1 -1.\n";
      cout << " > ";
      double xl, yl;
      cin >> xl >> yl;
      xres=static_cast<double>(w.width()-40)/xl;
      yres=static_cast<double>(w.height()-40)/yl;
      cout << "Resolution is (pixel/mm): " << xres << ' ' << yres << endl;
    }

    int listenSocket, connectSocket;
    unsigned short int listenPort = 22312;
    socklen_t clientAddressLength;
    struct sockaddr_in clientAddress, serverAddress;
    char line[256];
    listenSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (listenSocket < 0) {
      cerr << "cannot create listen socket";
      return 1;
    }
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddress.sin_port = htons(listenPort);
    if (bind(listenSocket,
           (struct sockaddr *) &serverAddress,
           sizeof(serverAddress)) < 0) {
      cerr << "cannot bind socket";
      return 1;
    }
    listen(listenSocket, 5);
    cout << "Waiting for TCP connection on port " << listenPort << " ...\n";
    clientAddressLength = sizeof(clientAddress);
    connectSocket = accept(listenSocket,
                           (struct sockaddr *) &clientAddress,
                           &clientAddressLength);
    if (connectSocket < 0) {
      cerr << "cannot accept connection ";
      return 1;
    }
    cout << "  connected to " << inet_ntoa(clientAddress.sin_addr);
    cout << ":" << ntohs(clientAddress.sin_port) << "\n";
    memset(line, 0x0, 256);
    int len=0;
    if (fullScreen)
      w.showFullScreen();
    else {
      w.show();
    }
    while ((len=recv(connectSocket, line, 255, 0)) > 0) {
      string message (line, len);
      int x1=-1;
      int y1=-1;
      cerr << "Receive message [" << message << "]\n";
      if (message.substr(0,4) == "quit") {
        break;
      } else if (message.substr(0,11) == "blackScreen") {
        w.setMode(MarkerWidget::blackScreen);
      } else if (message.substr(0,9) == "redScreen") {
        w.setMode(MarkerWidget::redScreen);
      } else if (message.substr(0,11) == "greenScreen") {
        w.setMode(MarkerWidget::greenScreen);
      } else if (message.substr(0,10) == "blueScreen") {
        w.setMode(MarkerWidget::blueScreen);
      } else if (message.substr (0,9) == "redMarker") {
        unsigned int p1 = message.find (' ', 0);
        unsigned int p2 = message.find (' ', p1+1);
        unsigned int sx = w.width()/20;
        unsigned int sy = w.height()/20;
        w.setPosition (sx*atoi(message.substr(p1+1, p2-p1-1).c_str()), w.height()-sy*atoi(message.substr(p2+1,message.length()).c_str()));
        w.setMode(MarkerWidget::redMarker);
      } else if (message.substr (0,17) == "calibrationMarker") {
        unsigned int p1 = message.find (' ', 0);
        unsigned int p2 = message.find (' ', p1+1);
        unsigned int sx = w.width()/20;
        unsigned int sy = w.height()/20;
        x1 = sx*atoi(message.substr(p1+1, p2-p1-1).c_str());
        y1 = sy*atoi(message.substr(p2+1,message.length()).c_str());
        w.setPosition (x1, w.height()-y1);
        w.setMode(MarkerWidget::calibrationMarker);
      } else {
        w.setMode(MarkerWidget::nextPosition);
      }
      a.processEvents();
      string sendmsg = "done.";
      if (x1>=0 && y1>=0) {
        stringstream inout;
        inout << "Marker " << x1/xres << ' ' << y1/yres << endl;
        getline (inout, sendmsg);
      }
      if (send(connectSocket, sendmsg.c_str(), sendmsg.length(), 0) < 0)
        cerr << "Error: cannot send receipt message";
      memset(line, 0x0, 256);
    }
    close(listenSocket);
    return 0;
  }catch(std::exception& e){
    std::cerr << "EXCEPTION: " << e.what() << std::endl;
    return -1;
  }
}
