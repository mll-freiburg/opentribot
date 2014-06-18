#include "RobotinoCam2.h"

using namespace rec::robotino::com;
using namespace Tribots;

RobotinoCam2::RobotinoCam2 ()
{
	std::cout << "Connecting camera... ";
	try
	{
		RobotinoCameraParameters param = com.cameraParameters();
		com.init();
		param.compression = COMPRESSION;
		param.resolution = (WIDTH == 640) ? VGA : QVGA;
		param.autoWithBalance = BALANCE;
		com.connectToHost(ADDRESS);
		com.setImageRequest(true);
		com.setCameraParameters(param);
		std::cout << "done." << std::endl;
	}
	
	catch (const rec::robotino::com::ComException& e)
		{ std::cerr << "Com Error: " << e.what() << std::endl; }
	catch (const std::exception& e)
		{ std::cerr << "Error: " << e.what() << std::endl; }
	catch (...)
		{ std::cerr << "Unknow Error" << std::endl; }
}

Tribots::ImageBuffer RobotinoCam2::getBuffer ()
{
	unsigned char data[BUFFER_SIZE];

	if (com.update()) {
		const RobotinoImage* image = com.lockImage();
		if (image != NULL) {
			if (image->parameters.type == JPG) {
				rec::core_lt::memory::ByteArray jpg(image->data, image->dataSize);
				std::cout << "JPEG compressed size: " << image->dataSize << std::endl;
				rec::core_lt::image::Image img = rec::core_lt::image::loadFromData(jpg, "jpg");
				memcpy(data, img.data(), BUFFER_SIZE);
			}
			else std::cout << "Received raw image. Yay?" << std::endl;
		}
		com.unlockImage();
	}

	Tribots::ImageBuffer *imgbuf = new Tribots::ImageBuffer(WIDTH, HEIGHT, ImageBuffer::FORMAT_RGB, data, BUFFER_SIZE);
	return *imgbuf;
}

