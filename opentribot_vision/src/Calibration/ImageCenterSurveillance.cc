
#include "ImageCenterSurveillance.h"
#include "centerRingOperation.h"
#include "../Formation/RGBImage.h"
#include "../../WorldModel/WorldModel.h"
#include "../../Structures/Journal.h"
#include <cmath>

using namespace Tribots;
using namespace std;

ImageCenterSurveillance::ImageCenterSurveillance (double centerX, double centerY) {
  setCenter (centerX, centerY);
  checkDone=true;
  wasStopped=true;
}

void ImageCenterSurveillance::setCenter (double centerX, double centerY) {
  cx=centerX;
  cy=centerY;
}

void ImageCenterSurveillance::check (const Image& image, GameState state) throw () {
  if (!state.in_game || state.refstate!=stopRobot) {
    wasStopped=false;
    return;
  }
  if (!wasStopped) {
    wasStopped=true;
    timestampLatestGameStopped.update();
    checkDone=false;
  }
  if (!checkDone && timestampLatestGameStopped.elapsed_msec()>=2000) {
    double neucx, neucy, neurmin, neurmax;
    RGBImage dummy (image.getWidth(), image.getHeight());
    bool success = findCenterRing (neucx, neucy, neurmin, neurmax, dummy, image, false);
//    unsigned int bx1, bx2, by1, by2;
//    determineBalanceArea (bx1, by1, bx2, by2, neucx, neucy, neurmin, neurmax);
    double d=sqrt((neucx-cx)*(neucx-cx)+(neucy-cy)*(neucy-cy));
    stringstream inout;
    inout << "ImageCenterSurveillance: ";
    if (success)
      inout << "No yellow circle found in image";
    else
      inout << "Circle center error distance is " << d;
    inout << endl;
    string message;
    getline (inout, message);
    JMESSAGETS (message.c_str());
    LOUT << message << '\n';
    checkDone=true;
  }
}
