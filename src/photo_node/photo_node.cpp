/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <photo/photo.h>
#include <self_test/self_test.h>

#include <photo/GetConfig.h>
#include <photo/SetConfig.h>
#include <photo/Capture.h>

class PhotoNode
{
public:
  photo_p photo_;
  photo_image_p photo_image_;
  boost::mutex photo_mutex_ ;

  ros::ServiceServer set_config_srv_;
  ros::ServiceServer get_config_srv_;
  ros::ServiceServer capture_srv_;

  PhotoNode() : photo_(NULL), photo_image_(NULL)
  {
    // initialize camera
    photo_image_ = photo_image_initialize();
    photo_ = photo_initialize();

    ros::NodeHandle private_nh("~");

    // autodetect a digital camera
    if(!photo_autodetect(photo_)) {
      ROS_FATAL("Error: could not open connection to photo.\n");
      private_nh.shutdown();
      return;
    }

    // ***** Start Services *****
    set_config_srv_ = private_nh.advertiseService("set_config", &PhotoNode::setConfig, this);
    get_config_srv_ = private_nh.advertiseService("get_config", &PhotoNode::getConfig, this);
    capture_srv_ = private_nh.advertiseService("capture", &PhotoNode::capture, this);
  }

  virtual ~PhotoNode()
  {
    // shutdown camera
    photo_close(photo_);
    photo_free(photo_);
  }

  bool setConfig(photo::SetConfig::Request& req, photo::SetConfig::Response& resp)
  {
    photo_mutex_.lock();
    bool ret = photo_set_config(photo_,req.param.c_str(),req.value.c_str());
    photo_mutex_.unlock();
    return ret;
  }

  bool getConfig(photo::GetConfig::Request& req, photo::GetConfig::Response& resp)
  {
    char* value = new char[255];
    photo_mutex_.lock();
    bool ret = photo_get_config(photo_,req.param.c_str(),&value);
    if(ret) resp.value = value;
    photo_mutex_.unlock();
    delete value;
    return ret;
  }

  bool capture(photo::Capture::Request& req, photo::Capture::Response& resp)
  {
    // capture a camera image
    photo_mutex_.lock();
    bool ret = photo_capture(photo_, photo_image_);
    if(ret) {
      // fill image message
      fillImage(resp.image, "rgb8", photo_image_->height, photo_image_->width, 3 * photo_image_->width, photo_image_->data);
    }
    photo_mutex_.unlock();
    return ret;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "photo");
  PhotoNode a;
  ros::spin();

  return 0;
}
