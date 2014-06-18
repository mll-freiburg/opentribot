
#include <iostream>
#include "ReferenceSystem.h"

using namespace TribotsTools;
using namespace std;

static string usage(string cmd)
{
  return "Usage: " + cmd + " config_file camera1.lut3d camera2.lut3d";
}

int main(int argc, char* argv[])
{
  if (argc != 4) {
    cerr << usage(argv[0]) << endl;
    exit(1);
  }

  ReferenceSystem rs(argc, argv);
  rs.run();
  
  return 0;
}
