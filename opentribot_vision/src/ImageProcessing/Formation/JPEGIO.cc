#include "JPEGIO.h"
#include "RGBImage.h"
#include <fstream>

extern "C" {
#include "jpeglib.h"               //  needs size_t
}


namespace Tribots {

  using std::istream;
  using std::ostream;
    
  class LibJPEGData {    
  public:
    struct jpeg_compress_struct cinfo;
    struct jpeg_decompress_struct dinfo;
    struct jpeg_error_mgr jerr;
  };


  #define BUFFER_SIZE 2048

  // jpeglib data source manager //////////////////////////////////////////////

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

  /////////////////////////////////////////////////////////////////////////////
				

  JPEGIO::JPEGIO(int quality)
  {
    lj = new LibJPEGData();
    
    lj->cinfo.err = jpeg_std_error(&lj->jerr);// use default error handler
    jpeg_create_compress(&lj->cinfo);       // create compression object

    lj->cinfo.input_components = 3;         // set color space
    lj->cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&lj->cinfo);          // default compression parameters
    jpeg_set_quality(&lj->cinfo, quality, TRUE); // set desired quality
    lj->cinfo.dct_method = JDCT_FLOAT;

    

    lj->dinfo.err = jpeg_std_error(&lj->jerr);// use default error handler
    jpeg_create_decompress(&lj->dinfo);     // create a decompression object
  }
  
  JPEGIO::~JPEGIO()
  {
    jpeg_destroy_compress(&lj->cinfo);
    delete lj;
  }

  void
  JPEGIO::write(const ImageBuffer& image, ostream& out) const
    throw(TribotsException)
  {
    bool newBuf = false;
    ImageBuffer* convBuf = NULL;
    if (image.format !=ImageBuffer::FORMAT_RGB) {
      convBuf = new ImageBuffer (image.width, image.height, 
                                 ImageBuffer::FORMAT_RGB, 
                                 new unsigned char[image.width * image.height * 3],
                                 image.width * image.height * 3);
      newBuf = true;
      ImageBuffer::convert(image, *convBuf);
    }
    const ImageBuffer imgBuf = (newBuf ? (*convBuf) : image);

    JPEGIO* nConstThis = const_cast<JPEGIO*>(this);

    JSAMPROW rowPointer[1];
    int rowStride = image.width * 3;   // Breite im Speicher
    
    setup_ostream_mgr(&nConstThis->lj->cinfo, out);

    nConstThis->lj->cinfo.image_width  = image.width;
    nConstThis->lj->cinfo.image_height = image.height;
    
    jpeg_start_compress(&nConstThis->lj->cinfo, TRUE);
    
    while (lj->cinfo.next_scanline < lj->cinfo.image_height) {
      rowPointer[0] = & imgBuf.buffer[lj->cinfo.next_scanline * rowStride];
      jpeg_write_scanlines(&nConstThis->lj->cinfo, rowPointer, 1);
    }    

    jpeg_finish_compress(&nConstThis->lj->cinfo);

    destroy_ostream_mgr(&nConstThis->lj->cinfo);

    if (newBuf) {  // Puffer selbst angelegt?
      delete [] convBuf->buffer;
      delete convBuf;
    }    
  }

  ImageBuffer* JPEGIO::read(ImageBuffer* image, istream& in) const
    throw(TribotsException)
  { 
    JPEGIO* nConstThis = const_cast<JPEGIO*>(this);
    setup_istream_mgr(&nConstThis->lj->dinfo, in);
    
    jpeg_read_header(&nConstThis->lj->dinfo, TRUE);
    jpeg_start_decompress(&nConstThis->lj->dinfo);

    if (image != 0) {
      if (image->width != (int)lj->dinfo.output_width || 
	  image->height != (int)lj->dinfo.output_height) {
	jpeg_abort_decompress(&nConstThis->lj->dinfo);
	destroy_istream_mgr(&nConstThis->lj->dinfo);
	throw TribotsException(__FILE__ ": Given image of wrong dimensions. "
			       "Please use a NULL-pointer instead, if the "
			       "image format is unknown.");
      }
    }
    else {
      image = new ImageBuffer (lj->dinfo.output_width, lj->dinfo.output_height, ImageBuffer::FORMAT_RGB, new unsigned char [3*lj->dinfo.output_width* lj->dinfo.output_height], 3*lj->dinfo.output_width* lj->dinfo.output_height);
    }

    // create temporary input buffer, if format of image is not RGB
    // \TODO: change color space in decompress info to YUV if necessary and let
    //        the jpeg library do the conversion work!
    ImageBuffer *readBuffer;
    if (image->format == ImageBuffer::FORMAT_RGB) { 
      readBuffer = image;  // can use original buffer
    }
    else {
      readBuffer = new ImageBuffer(lj->dinfo.output_width, lj->dinfo.output_height, 
				   ImageBuffer::FORMAT_RGB, 
				   new unsigned char[lj->dinfo.output_width * 
						     lj->dinfo.output_height * 3],
				   lj->dinfo.output_width * 
				   lj->dinfo.output_height * 3);
    }    


    JSAMPROW rowPointer[1];
    int rowStride = readBuffer->width * 3;   // width in memory

    while (lj->dinfo.output_scanline < lj->dinfo.output_height) { // read scanlines
      rowPointer[0] = readBuffer->buffer + (rowStride * lj->dinfo.output_scanline);
      jpeg_read_scanlines(&nConstThis->lj->dinfo, 
			  rowPointer,
			  1);
    }
    jpeg_finish_decompress(&nConstThis->lj->dinfo); // clean up lj->dinfo for reuse

    // if the image is not an RGB image, a conversion is needed. clean up the
    // temporary rgb read buffer.
    if (image->format != ImageBuffer::FORMAT_RGB) {//need conversion
      ImageBuffer::convert(*readBuffer, *image);
      delete [] readBuffer->buffer; // delete the read buffer (has been copied)
      delete readBuffer;            // read buffer was dynamically created
    }
    destroy_istream_mgr(&nConstThis->lj->dinfo);
    return image;
  }


}
