#include <stdio.h>
#include <zlib.h>
#include <iostream>
#include <jpeg-utils/jpeg-utils.h>
#include "data_capture.hpp"

namespace fovis_lcm
{

DataCapture::DataCapture(lcm::LCM &lcm, const std::string& kinect_lcm_channel)
{
  width = 640;
  height = 480;

  rgb_params.width = width;
  rgb_params.height = height;

  // TODO read these values from param / camera.
  rgb_params.fx = 528.49404721;
  rgb_params.fy = rgb_params.fx;
  rgb_params.cx = 319.50000000; // From the renderer. 
  rgb_params.cy = 239.50000000; // From the renderer. 

  depth_image = new fovis::DepthImage(rgb_params, width, height);
  depth_data = new float[width * height];
  depth_buf = new uint8_t[width * height];
  uncompress_depth_buffer_size = width * height;
  gray_buf = new uint8_t[width * height];
  rgb_buf = new uint8_t[width * height * 3];
  lcm.subscribe(kinect_lcm_channel, &DataCapture::onKinectMessage, this);
}

DataCapture::~DataCapture()
{
  delete[] depth_data;
  delete[] gray_buf;
  delete depth_image;
}

void DataCapture::onKinectMessage(const lcm::ReceiveBuffer* rbuf, 
				  const std::string& channel,
				  const kinect::frame_msg_t *msg){
  // Decode and copy the data.
  processDepth(msg->depth);
  processImage(msg->image);
}    

void
DataCapture::processDepth(const kinect::depth_msg_t &depth)
{
  
  int npixels = width * height;
  
  if(depth.compression != kinect::depth_msg_t::COMPRESSION_NONE) {
    if(depth.uncompressed_size > uncompress_depth_buffer_size) {
      uncompress_depth_buffer_size = depth.uncompressed_size;
      delete depth_buf;
      std::cout << "Reallocating the depth buffer" << std::endl;
      depth_buf = new uint8_t[uncompress_depth_buffer_size];
      // //(uint8_t*) realloc(self->uncompress_buffer, self->uncompress_buffer_size);
    }
    unsigned long dlen = depth.uncompressed_size;
    int status = uncompress(depth_buf, &dlen, depth.depth_data.data(),
			    depth.depth_data_nbytes);
    if(status != Z_OK) {
      return;
    }
  } else {
    memcpy(depth_buf, depth.depth_data.data(), npixels);
  }
  
  switch(depth.depth_data_format) {
  case kinect::depth_msg_t::DEPTH_11BIT:
    {
      std::cout << "Depth_11BIT mode not supported yet - fix" << std::endl;
      uint16_t* rdd = (uint16_t*) depth_buf;
      int i;
      for(i=0; i<npixels; i++) {
	const uint16_t& d = rdd[i];
	// this is disparity data.
	if(d != 0){
	  depth_data[i] = d * 1.0e-3;
	}
	else {
	  depth_data[i] = NAN;
	}
      }
    }
    break;
  case kinect::depth_msg_t::DEPTH_MM:
    {
      uint16_t* rdd = (uint16_t*) depth_buf;
      int i;
      for(i=0; i<npixels; i++){
	const uint16_t& d = rdd[i];
	// TODO(sachih): Depth dropouts should be set to NAN - how is this encoded?
	if(d != 0){
	  depth_data[i] = 1.0e-3 * d;
	} else{
	  depth_data[i] = NAN;
	}	
      }
    }
    break;
  case kinect::depth_msg_t::DEPTH_10BIT:
    fprintf(stderr, "10-bit depth data not supported\n");
    return;
  default:
    fprintf(stderr, "Unhandled type\n");
    return;
  }

  // uint16_t* depth_mm = (uint16_t*)data;
  // int num_pixels = width * height;
  // for(int i=0; i<num_pixels; i++) {
  //   uint16_t d = depth_mm[i];
  //   if(d != 0) {
  //     depth_data[i] = d * 1e-3;
  //   } else {
  //     depth_data[i] = NAN;
  //   }
  // }
  depth_image->setDepthImage(depth_data);
  have_depth = true;
  // Do we need to regalign the depth?
}

void
DataCapture::processImage(const kinect::image_msg_t &image)
{
  if(image.image_data_format == kinect::image_msg_t::VIDEO_RGB) {
    memcpy(rgb_buf, image.image_data.data(), 
	   width * height * 3);
  } else if(image.image_data_format == kinect::image_msg_t::VIDEO_RGB_JPEG) {
    jpeg_decompress_8u_rgb(image.image_data.data(), image.image_data_nbytes,
			   rgb_buf, width, height, width * 3);
  } else {
    // Do we get gray images??
    std::cout << "Error decoding image" << std::endl;
    return;
  }
  const int num_pixels = width * height;
  uint8_t* rgb_pixel = rgb_buf;
  uint8_t* gray_pixel = gray_buf;
  for(int i=0; i<num_pixels; i++) {
    gray_pixel[0] = (rgb_pixel[0] + rgb_pixel[1] + rgb_pixel[2]) / 3;
    gray_pixel++;
    rgb_pixel += 3;
  }
  have_image = true;
}

}
