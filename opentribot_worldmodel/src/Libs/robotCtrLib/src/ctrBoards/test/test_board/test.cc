#include "Tmc200.h"
#include <fstream>

int main(int argc, char **argv)
{
  std::ostream *errstream = new std::ofstream("err.log");
  std::ostream *infstream = new std::ofstream("inf.log");
  
  Tmc200 board("OmniRobot.cfg", errstream, infstream);
  bool kicking_test = false;


  if (argc > 1 && !strcmp(argv[1],"k")) kicking_test = true;

  if (board.init())
    {
      std::cout << "Board init: ... OK! \n";

      float v;
      if (board.getPowerSupplyVoltage(v))
	std::cout << "Versorgungsspannung: " << v << "V \n";
      else
	std::cout << "Fehler bei Abfrage der Versorgungsspannung!\n";


      std::cout << "Try to move ... BLOCKING COMMUNICATION \n";
      tmc_real_data_t data;
      for (int i=0; i<20; i++)
	{
	  board.setVelocityRPS(5,5,5, data);
	  data.print(std::cout);
	  std::cout << "\n.\n";
	  usleep(20000);
	}
      usleep(1000000);
      std::cout << " \n reverse \n";
      for (int i=0; i<20; i++)
	{
	  board.setVelocityRPS(-5,-5,-5, data);
	  data.print(std::cout);
	  std::cout << "\n.\n";
	  usleep(20000);
	}
      std::cout << "\nready with moving!\n";

      if (kicking_test) {
	sleep(5);
	std::cout << "\nkicking (30ms) ... \n";
	board.set_port1L(true);
	usleep(100000);
	board.set_port1L(false);
	sleep(1);
	std::cout << "\nkicking (100ms) ... \n";
	board.set_port1L(true);
	usleep(30000);
	board.set_port1L(false);
	sleep(1);
	std::cout << "\nkicking (5s) ... \n";
	board.set_port1L(true);
	usleep(5000000);
	board.set_port1L(false);
      }
    }
  else
    {
      std::cout << "Initialisierung Borad Fehlgeschlagen! Näheres in err.log und info.log\n";
    }

}
