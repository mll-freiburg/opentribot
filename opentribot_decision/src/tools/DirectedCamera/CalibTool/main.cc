#include "CalibTool.h"

using namespace Tribots;
using namespace std;

int main(int argc, char* argv[])
{

  if (argc!=4 && argc != 5)
  {
    CalibTool::usage();
    return -1;
  }
  try{
    CalibTool calib(argc, argv);
  }catch(Tribots::TribotsException& e){
    cerr << e.what() << "\n\r" << flush;
  }catch(std::exception& e){
    cerr << e.what() << "\n\r" << flush;
  }
  return 0;
}
