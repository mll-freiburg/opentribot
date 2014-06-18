#ifndef _ROBOTINOCAM2_H_
#define _ROBOTINOCAM2_H_

#include "rec/robotino/com/all.h"
#include "rec/core_lt/Image.h"
#include "rec/core_lt/utils.h"
#include "rec/core_lt/Timer.h"
#include "rec/core_lt/image/conv.h"
#include "robotinocom/robotinocom.h"
#include "robotinocom/camerastuff.h"
#include "../ImageSource.h"
#include "../../../Structures/TribotsException.h"
#include "../../../Fundamental/ConfigReader.h"

#define WIDTH 640
#define HEIGHT 480
#define BUFFER_SIZE WIDTH*HEIGHT*3

#define BALANCE false
#define COMPRESSION LowCompression
#define ADDRESS "172.31.3.11"
//#define ADDRESS "172.26.1.1"

namespace Tribots {	class 
#ifdef rec_robotino_com2_EXPORTS
__declspec(dllexport)
#endif
	RobotinoCam2
	{
	public:
		RobotinoCam2 ();
		Tribots::ImageBuffer getBuffer ();
		
	protected:
		rec::core_lt::Timer imageUpdateTimer;
		RobotinoCom com;
	};
}

#endif
