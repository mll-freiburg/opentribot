#include "VideoDebug.h"
#include "fstream"

using namespace std;

using namespace cvtk;



//////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                          JPEG
//
//////////////////////////////////////////////////////////////////////////////////////////////////////

// read routine is taken from Tribots code..

extern "C"{

#include "jpeglib.h"
}

#define BUFFER_SIZE 2048


class LibJPEGData {    
public:
	struct jpeg_compress_struct cinfo;
	struct jpeg_decompress_struct dinfo;
	struct jpeg_error_mgr jerr;
};

struct jpeg_istream_mgr {
	struct jpeg_source_mgr srcMgr;
	istream* in;
	JOCTET buffer[BUFFER_SIZE];
};

typedef jpeg_istream_mgr * istream_mgr_ptr;

static void init_istream(j_decompress_ptr) 
{}

/** Fills the input buffer from the input stream. Resets the 
 * next_input_byte pointer to the beginning of the buffer. */
static boolean fill_input_buffer (j_decompress_ptr dinfo)
{
	// get the istream manager struct
	istream_mgr_ptr mgr = reinterpret_cast<istream_mgr_ptr>(dinfo->src);

	mgr->in->read(reinterpret_cast<char*>(&mgr->buffer[0]), BUFFER_SIZE);

	mgr->srcMgr.next_input_byte = &mgr->buffer[0];  // reset buffer
	mgr->srcMgr.bytes_in_buffer = BUFFER_SIZE;

	return TRUE;
}

/** Skips data in the input stream and refills the buffer as necessary.
 *  Implemented with a simple recursion. */
static void skip_input_data(j_decompress_ptr dinfo, long num_bytes)
{
	if (num_bytes <= 0) {    // error -> treat as no-op
		return;
	}
	// get the istream manager struct
	istream_mgr_ptr mgr = reinterpret_cast<istream_mgr_ptr>(dinfo->src);


	if (num_bytes <= (long)mgr->srcMgr.bytes_in_buffer) {// Verankerung: Genug
		mgr->srcMgr.bytes_in_buffer -= num_bytes;//Daten? Sonst weiterbewegen im 
		mgr->srcMgr.next_input_byte += num_bytes;//Puffer
	}
	else {                                    // Nicht genug Daten:
		num_bytes -= mgr->srcMgr.bytes_in_buffer;//vorhandene Daten ueberspringen
		fill_input_buffer(dinfo);               // den Puffer neu fuellen und
		skip_input_data(dinfo, num_bytes);      // rekursiv den Rest abarbeiten
	}
}

static void destroy_istream(j_decompress_ptr)
{}

static void setup_istream_mgr(j_decompress_ptr dinfo, istream& in)
{
	istream_mgr_ptr mgr = new jpeg_istream_mgr();
	mgr->in = &in;
	mgr->srcMgr.next_input_byte   = &mgr->buffer[0];
	mgr->srcMgr.bytes_in_buffer   = 0;

	mgr->srcMgr.init_source       = &init_istream;
	mgr->srcMgr.fill_input_buffer = &fill_input_buffer;
	mgr->srcMgr.skip_input_data   = &skip_input_data;
	mgr->srcMgr.resync_to_restart = &jpeg_resync_to_restart;
	mgr->srcMgr.term_source       = &destroy_istream;

	dinfo->src = reinterpret_cast<struct jpeg_source_mgr*>(mgr);
} 

static void destroy_istream_mgr(j_decompress_ptr dinfo)
{
	istream_mgr_ptr mgrPtr = reinterpret_cast<istream_mgr_ptr>(dinfo->src);
	delete mgrPtr;
	dinfo->src = NULL;
}

// jpeglib data destination manager /////////////////////////////////////////

struct jpeg_ostream_mgr {
	struct jpeg_destination_mgr destMgr;
	ostream* out;
	JOCTET buffer[BUFFER_SIZE];
};  

typedef jpeg_ostream_mgr * ostream_mgr_ptr;

static void init_ostream(j_compress_ptr) 
{}

static boolean empty_output_buffer(j_compress_ptr cinfo) 
{
	// get the ostream manager struct
	ostream_mgr_ptr mgr = reinterpret_cast<ostream_mgr_ptr>(cinfo->dest);

	mgr->out->write(reinterpret_cast<char*>(&mgr->buffer[0]), 
			BUFFER_SIZE);      // write buffer

	mgr->destMgr.next_output_byte = &mgr->buffer[0];  // reset buffer
	mgr->destMgr.free_in_buffer   = BUFFER_SIZE;

	return TRUE;
}

static void destroy_ostream(j_compress_ptr cinfo)
{
	ostream_mgr_ptr mgr = reinterpret_cast<ostream_mgr_ptr>(cinfo->dest);

	mgr->out->write(reinterpret_cast<char*>(&mgr->buffer[0]), 
			BUFFER_SIZE-mgr->destMgr.free_in_buffer); //flush
	mgr->out->flush();
}

static void setup_ostream_mgr(j_compress_ptr cinfo, ostream& out)
{
	ostream_mgr_ptr mgr = new jpeg_ostream_mgr();
	mgr->out = &out;
	mgr->destMgr.next_output_byte = &mgr->buffer[0];
	mgr->destMgr.free_in_buffer = BUFFER_SIZE;

	mgr->destMgr.init_destination    = &init_ostream;
	mgr->destMgr.empty_output_buffer = &empty_output_buffer;
	mgr->destMgr.term_destination    = &destroy_ostream;

	cinfo->dest = reinterpret_cast<struct jpeg_destination_mgr*>(mgr);
} 

static void destroy_ostream_mgr(j_compress_ptr cinfo)
{
	ostream_mgr_ptr mgrPtr = reinterpret_cast<ostream_mgr_ptr>(cinfo->dest);
	delete mgrPtr;
	cinfo->dest = NULL;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////


VideoDebug::VideoDebug()
{
}

VideoDebug::~VideoDebug()
{
}

unsigned char* VideoDebug::YUV420_to_RGB(const unsigned char* yuv420,int width, int height)
{
	unsigned char* buf=new unsigned char[width*height*2]; // yuv422

	VideoUtils::yuv420p_to_yuv422(yuv420, buf, width,height);

	unsigned char* rgbbuf=new unsigned char[width*height*3]; // rgb

	// now convert to rgb
	int totalsize=width*height;

	PixelOperations::yuv422pkt* yuvptr = (PixelOperations::yuv422pkt*)buf;
	PixelOperations::rgb2pkt* rgbptr = (PixelOperations::rgb2pkt*)rgbbuf;

	for (int i=0;i<totalsize;i+=2,yuvptr++,rgbptr++)
	{
		int r1,g1,b1,r2,g2,b2,u,v;

		u = (int)(yuvptr->u-128);
		v = (int)(yuvptr->v-128);
		PIXEL_YUV2RGB((int)yuvptr->y1,u,v, r1,g1,b1);
		PIXEL_YUV2RGB((int)yuvptr->y2,u,v, r2,g2,b2);
		rgbptr->r1=min(max(r1,0),255);
		rgbptr->g1=min(max(g1,0),255);
		rgbptr->b1=min(max(b1,0),255);
		rgbptr->r2=min(max(r2,0),255);
		rgbptr->g2=min(max(g2,0),255);
		rgbptr->b2=min(max(b2,0),255);
	}

	delete[] buf;

	// return the image
	return rgbbuf;
}

void VideoDebug::writePPM(unsigned char* rgbbuf, int width, int height, const char* filename)
{
	FILE* ppmfile;
	char head[500];

	ppmfile = fopen(filename,"wb");
	sprintf(head,"P6\n%d %d 255\n",width,height);
	fwrite((unsigned char*)&head,strlen(head),sizeof(unsigned char),ppmfile);
	fwrite((unsigned char*)rgbbuf,width*height*3,sizeof(unsigned char),ppmfile);

	fclose(ppmfile);
}

void VideoDebug::writePPM_YUV_Half(const unsigned char* yuvbuf, int width, int height, const char* filename)
{
	FILE* ppmfile;
	char head[500];

	int target_width  = width/2;
	int target_height = height/2;
	int target_num_pixels = target_width * target_height;

	const unsigned char* yptr = &yuvbuf[0];
	const unsigned char* uptr = &yuvbuf[target_num_pixels*4];    
	const unsigned char* vptr = &yuvbuf[target_num_pixels*5];

	RGBValue rgb; YUVValue yuv;


	unsigned char* outBuf = new unsigned char[target_num_pixels*3];
	memset(outBuf, 0, target_num_pixels*3);
	RGBValue* tptr = reinterpret_cast<RGBValue*>(&outBuf[0]);

	// convert YUV 4:2:0 to half-size RGB
	for (int y=0; y < target_height; y++, yptr += target_width*2)
	{
		for (int x=0; x < target_width; x++, yptr += 2, uptr++, vptr++)
		{
			yuv.y = *yptr; yuv.u = *uptr; yuv.v = *vptr;
			PixelConversion::convert(yuv, &rgb);
			*tptr++ = rgb;
		}
	}	

	ppmfile = fopen(filename,"wb");
	sprintf(head,"P6\n%d %d 255\n",target_width,target_height);
	fwrite((unsigned char*)&head,strlen(head),sizeof(unsigned char),ppmfile);
	fwrite((unsigned char*)outBuf,target_width*target_height*3,sizeof(unsigned char),ppmfile);

	fclose(ppmfile);

	delete[] outBuf;
}




void VideoDebug::loadPPM_YUV_Half(unsigned char* yuvbuf, int width, int height, const char* filename)
{


	fstream in (filename);
	if (!in)
	{
		cerr << "Could not read PPM from file: " << filename << endl;
		return;
	}
	string s;
	in >> s;
	if (!(s == "P6"))
	{
		cerr << "Not a PPM image" << endl;
		in.close ();
		return;
	}

	int maxval;
	in >> width >> height >> maxval;

	if (width !=320 || height != 240){
		cerr<< "PPM IMage not half-size"<<endl;
		in.close();
		return ;
	}
	in.ignore (1);		// Zeilenwechsel ignorieren
	RGBValue rgb;
	YUVValue yuv;

	int target_num_pixels=width*height;


	unsigned char* yptr = &yuvbuf[0];
	unsigned char* uptr = &yuvbuf[4*target_num_pixels];    
	unsigned char* vptr = &yuvbuf[target_num_pixels*5];



	for (int j=0;j<height;j++){
		for(int i=0;i<width;i++){
			in.read((char*)&rgb,3);
			PixelConversion::convert(rgb,&yuv);
			*yptr++=yuv.y;
			*yptr++=yuv.y;
			*uptr++=yuv.u;
			*vptr++=yuv.v;
		}
		yptr+=width*2;
	}

	in.close();
	return ;

}


void VideoDebug::loadJPG_YUV_Half(unsigned char* yuvbuf, int width, int height, const char* filename)
{
	ifstream infile(filename);
	
	if (infile.good())
	{
		//////////////////// startup
		LibJPEGData* lj = new LibJPEGData();
	
		lj->cinfo.err = jpeg_std_error(&lj->jerr);// use default error handler
		jpeg_create_compress(&lj->cinfo);       // create compression object
	
		lj->cinfo.input_components = 3;         // set color space
		lj->cinfo.in_color_space = JCS_RGB;
	
		jpeg_set_defaults(&lj->cinfo);          // default compression parameters
		jpeg_set_quality(&lj->cinfo, 50, TRUE); // set desired quality
		lj->cinfo.dct_method = JDCT_FLOAT;
	
		lj->dinfo.err = jpeg_std_error(&lj->jerr);// use default error handler
		jpeg_create_decompress(&lj->dinfo);     // create a decompression object
	
		/////////////////////// do work
		setup_istream_mgr(&lj->dinfo, infile);
	
		jpeg_read_header(&lj->dinfo, TRUE);
		jpeg_start_decompress(&lj->dinfo);
	
		int width,height;
		width = lj->dinfo.output_width;
		height = lj->dinfo.output_height;
		
		if (width != 320 || height != 240)
		{
			std::cerr << "ERROR: wrong image dimensions! (should be 320x240, got " << width << "x" << height << ")\n";
			return;
		}
		
		// create buffer
		unsigned char* jpegbuf =  new unsigned char [3*width*height];
	
		JSAMPROW rowPointer[1];
		int rowStride = width * 3;   // width in memory
	
		while (lj->dinfo.output_scanline < height) { // read scanlines
			rowPointer[0] = jpegbuf + (rowStride * lj->dinfo.output_scanline);
			jpeg_read_scanlines(&lj->dinfo, rowPointer, 1);
		}
		jpeg_finish_decompress(&lj->dinfo); // clean up lj->dinfo for reuse
	
		destroy_istream_mgr(&lj->dinfo);
	
		infile.ignore(1);// Zeilenwechsel ignorieren
		RGBValue rgb;
		YUVValue yuv;
	
		int target_num_pixels=width*height;
	
		unsigned char* yptr = &yuvbuf[0];
		unsigned char* uptr = &yuvbuf[4*target_num_pixels];    
		unsigned char* vptr = &yuvbuf[target_num_pixels*5];
	
		RGBValue* rgb_ptr = (RGBValue*)jpegbuf;
		
		for (int j=0;j<height;j++){
			for(int i=0;i<width;i++){
				PixelConversion::convert(*rgb_ptr,&yuv);
				*yptr++=yuv.y;
				*yptr++=yuv.y;
				*uptr++=yuv.u;
				*vptr++=yuv.v;
				rgb_ptr++;
			}
			yptr+=width*2;
		}
	
		//////////////////////// die
	
		jpeg_destroy_compress(&lj->cinfo);
		delete lj;
		delete[] jpegbuf;
	} else
	{
		std::cerr << "ERROR: file " << filename << " not found!\n";
	}
}


