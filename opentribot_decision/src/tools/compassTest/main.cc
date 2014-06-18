#include "compass.h"
#include <string>
#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
  if (argc < 2 || argc > 3 || 
      (argc == 3 && argv[2][0] != 'e' && argv[2][0] != 'v')) {
    cerr << "Usage: " << argv[0] << " serial_port [e|v]" << endl;
    exit(1);
  }
  
  int fd = compass_init(argv[1]);

  double roll, pitch, yaw;
  struct compass_vector mag, accel, gyro;

  while (1) {
    if (argc == 3 && argv[2][0] == 'v') {
      if (compass_getVectors(fd, &mag, &accel, &gyro) != 0) {
	cerr << "Konnte nicht lesen!" << endl;
	exit(1);
      } 
      cerr << "Magnetfeldvektor: " << mag.x << " " << mag.y << " " << mag.z 
	   << " Beschl.: " << accel.x << " " << accel.y << " " << accel.z 
	   << " Winkelgeschw.: " << gyro.x << " " << gyro.y << " " << gyro.z 
	   << endl; 
    }
    else {
      if (compass_getEuler(fd, &roll, &pitch, &yaw) != 0) {
	cerr << "Konnte nicht lesen!" << endl;
	exit(1);
      } 
      cerr << "Roll: " << roll << " Pitch: " << pitch << " Yaw: "  << yaw 
	   << endl; 
    }
  }

  close(fd);
  
  return 0;
}
