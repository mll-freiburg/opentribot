#ifndef _BBCLIENT_H_
#define _BBCLIENT_H_


#include "../bbcomm.h"
#include "sys/time.h"
#include "stdlib.h"
#include "stdio.h"
#include <string>

using namespace std;

class BBClient{
public:

BBComm *bbcomm;
int init(std::string  hostname );

int
update ();




};


#endif
