#include "SharedMemWriter.h"

#include <sstream>

namespace Tribots {

  using namespace std;
  
  SharedMemWriter::SharedMemWriter(ImageIO* imageIO, const std::string& filename,
			   int step, bool singleFile)
    : ImageMonitor(), imageIO(imageIO), logOut(0), imgOut(0), 
      filename(filename), singleFile(singleFile), counter(0), step(step)
  {
  

	sharedMemory=new QSharedMemory("foobar");
	sharedMemory->create(640*480*2);



  }

  SharedMemWriter::~SharedMemWriter()
  {
    sharedMemory->unlock();
    delete sharedMemory;

  }

  void SharedMemWriter::monitor(const ImageBuffer& image, 
			    const Time&, const Time&)
  {
	sharedMemory->lock();
	char *to = (char*)sharedMemory->data();
	cout <<"Size::"<<image.size<<endl;
	memcpy(to, image.buffer,image.size);
	sharedMemory->unlock();  

  }


}
