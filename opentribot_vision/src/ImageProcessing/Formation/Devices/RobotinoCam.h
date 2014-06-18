#ifndef _ROBOTINOCAM_H_
#define _ROBOTINOCAM_H_

#include "rec/robotino/com/all.h"
#include "rec/robotino/com/Camera.h"
#include "rec/core_lt/Image.h"
#include "rec/core_lt/utils.h"
#include "rec/core_lt/Timer.h"
#include "rec/core_lt/image/conv.h"
#include "../ImageSource.h"
#include "../../../Structures/TribotsException.h"
#include "../../../Fundamental/ConfigReader.h"

#define WIDTH 320
#define HEIGHT 240

#define BUFFER_SIZE WIDTH*HEIGHT*3

namespace Tribots {	class 
#ifdef rec_robotino_com2_EXPORTS
__declspec(dllexport)
#endif
	RobotinoCam : public rec::robotino::com::Camera
	{
	public:
		RobotinoCam ();
		
		void imageReceivedEvent (const unsigned char* data, unsigned int dataSize, rec::robotino::com::Camera::ImageType_t type);
		Tribots::ImageBuffer getBuffer ();
		
	protected:
		unsigned char buffer[BUFFER_SIZE];
		rec::core_lt::Timer imageUpdateTimer;
		rec::robotino::com::Com com;
	};
}

#endif
