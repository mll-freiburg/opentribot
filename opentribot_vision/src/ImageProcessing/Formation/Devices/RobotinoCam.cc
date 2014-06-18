#include "RobotinoCam.h"

using namespace rec::robotino::com;
using namespace Tribots;

RobotinoCam::RobotinoCam ()
{
/*
	unsigned char data[640*480*3];
	buffer = * new ImageBuffer(640, 480, ImageBuffer::FORMAT_RGB, data, 640*480*3);
*/
	std::cout << "Connecting camera... ";
	try
	{
		this->setComId(com.id());
		this->setResolution(WIDTH == 640 ? rec::robotino::com::Camera::VGA : rec::robotino::com::Camera::QVGA);

		this->setStreaming(true);
	
		com.setAddress("172.26.1.1");
		com.connect();
		
		std::cout << "done." << std::endl;
	}
	
	catch (const rec::robotino::com::ComException& e)
		{ std::cerr << "Com Error: " << e.what() << std::endl; }
	catch (const std::exception& e)
		{ std::cerr << "Error: " << e.what() << std::endl; }
	catch (...)
		{ std::cerr << "Unknow Error" << std::endl; }
}

void RobotinoCam::imageReceivedEvent (const unsigned char* imageData, unsigned int imageDataSize, ImageType_t type)
{
	rec::core_lt::memory::ByteArray jpg(imageData, imageDataSize);
	//std::cout << "Compressed bytes: " << imageDataSize << std::endl;
    rec::core_lt::image::Image img = rec::core_lt::image::loadFromData(jpg, "jpg");
	std::cout << "Step: " << img.step() << std::endl;
	if (img.step() != 0)
		memcpy(buffer, img.data(), BUFFER_SIZE);
	std::cout << imageUpdateTimer.msecsElapsed() << std::endl;
	imageUpdateTimer.start();
}

Tribots::ImageBuffer RobotinoCam::getBuffer ()
{
	unsigned char data[BUFFER_SIZE];
	memcpy(data, buffer, BUFFER_SIZE);
	ImageBuffer *imgbuf = new ImageBuffer(WIDTH, HEIGHT, ImageBuffer::FORMAT_RGB, data, BUFFER_SIZE);
	return *imgbuf;
}

