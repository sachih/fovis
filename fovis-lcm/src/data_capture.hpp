#ifndef __fovis_lcm_data_capture_freenect_hpp__
#define __fovis_lcm_data_capture_freenect_hpp__

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/kinect/frame_msg_t.hpp>
#include <fovis/fovis.hpp>

namespace fovis_lcm
{

class DataCapture
{
  public:
    DataCapture(lcm::LCM &lcm, const std::string& kinect_lcm_channel);
    ~DataCapture();

    fovis::DepthImage* getDepthImage() {
      return depth_image;
    }

    const fovis::CameraIntrinsicsParameters& getRgbParameters() const {
      return rgb_params;
    }

    const uint8_t* getGrayImage() {
      return gray_buf;
    }
  
  bool hasData() const {
    return have_depth && have_image;
  }

  void expireData(){
    have_depth = false;
    have_image = false;
  }

  private:
    void onKinectMessage(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
				const kinect::frame_msg_t *msg);
    void processDepth(const kinect::depth_msg_t &depth_message);
    void processImage(const kinect::image_msg_t &image_message);

    fovis::DepthImage* depth_image;

    int width;
    int height;

    fovis::CameraIntrinsicsParameters rgb_params;

    bool have_image;
    bool have_depth;

  int uncompress_depth_buffer_size;
  uint8_t* depth_buf;
  float *depth_data;
  

  uint8_t* gray_buf;
  uint8_t* rgb_buf;
};

}

#endif
