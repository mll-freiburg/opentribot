
#include "TmcCanCom.h"
#include <iostream>
#include <sys/time.h>

using namespace std;

int main() {
  timeval t1, t2;
  double diff;

  unsigned int confec;
  RobotCtr2::TmcCanCom* com = 0;
  
  try {
    com = new RobotCtr2::TmcCanCom();    
  }
  catch (RobotCtr2::TmcCanException& e) {
    std::cout << "We have an exception: " << e.what() << "\n";
    exit(0);
  }
  
  bool res;
  std::string version;

  for (int i=0; i<100; i++) {
    gettimeofday(&t1, 0);
    res=com->ping(&version);
    gettimeofday(&t2, 0);
    
    diff = (t2.tv_sec - t1.tv_sec) * 1000.0 + (t2.tv_usec - t1.tv_usec) / 1000.0;
    if (res) std::cout << " Success " << version << " time: " << diff << "ms \n";
  }

  delete com;
}
