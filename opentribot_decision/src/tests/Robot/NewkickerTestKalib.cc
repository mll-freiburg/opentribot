
#include  <iostream>
#include <cmath>

using namespace std;


int main(int argc, char* argv[]) {
	
	double distance_mm = atof(argv[1]) / 1000;
	unsigned char klength_ms = (unsigned char) atoi( (argv[2]) );

		double x_max_params[3];
		double y_max_params[3];
		x_max_params[0] = 0.835;
		x_max_params[1] = 15.217;
		x_max_params[2] = 0.66;
		y_max_params[0] = 1.39;
		y_max_params[1] = -2.26;
		y_max_params[2] = -3.41;
	
		klength_ms = klength_ms > 60 ? 60 : klength_ms;
		klength_ms = klength_ms < 16 ? 16 : klength_ms;

	
  	double x_max = x_max_params[0] * log( (double) klength_ms - x_max_params[1]) + x_max_params[2];
	double y_max = y_max_params[0] * log( (double) klength_ms - y_max_params[1]) + y_max_params[2];
	
	double res =  -(y_max / (x_max * x_max)) * (distance_mm - x_max)*(distance_mm - x_max) + y_max;
	
	res = (res < 0) ? 0 : res;
	cout <<  res  << endl;
}
