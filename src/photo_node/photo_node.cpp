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

// ROS Headers
#include <ros/ros.h>
#include <self_test/self_test.h>

// ROS Messages
#include <sensor_msgs/fill_image.h>

// ROS Services
#include <photo/GetConfig.h>
#include <photo/SetConfig.h>
#include <photo/Capture.h>

// photo library headers
#include "photo/photo_camera_list.hpp"
#include "photo/photo_camera.hpp"
#include "photo/photo_image.hpp"

#include "boost/thread.hpp"


class PhotoNode
{
public:
  photo_camera_list camera_list_;
  photo_camera camera_;
  photo_image image_;

  boost::mutex photo_mutex_ ;
  boost::thread triggered_thread_;

  ros::ServiceServer set_config_srv_;
  ros::ServiceServer get_config_srv_;
  ros::ServiceServer capture_srv_;

  ros::Publisher triggered_capture_;

  PhotoNode() :
    camera_list_(),
    camera_(),
    image_()
  {

    ros::NodeHandle private_nh("~");
    GPContext* private_context;

    // initialize camera

    // create context
    private_context = camera_.photo_camera_create_context();

    // autodetect all cameras connected
    if( camera_list_.autodetect( private_context ) == false )
    {
      ROS_FATAL( "photo_node: Autodetection of cameras failed." );
      gp_context_unref( private_context );
      private_nh.shutdown();
      return;
    }

    // open camera from camera list
    if( camera_.photo_camera_open( &camera_list_, 0 ) == false )
    {
      ROS_FATAL( "photo_node: Could not open camera %d.", 0 );
      gp_context_unref( private_context );
      private_nh.shutdown();
      return;
    }

    // ***** Start Services *****
    set_config_srv_ = private_nh.advertiseService("set_config", &PhotoNode::setConfig, this);
    get_config_srv_ = private_nh.advertiseService("get_config", &PhotoNode::getConfig, this);
    capture_srv_ = private_nh.advertiseService("capture", &PhotoNode::capture, this);

    // Advertise topic
    triggered_capture_ = private_nh.advertise<sensor_msgs::Image>("triggered_capture",1);

    triggered_thread_ = boost::thread(boost::bind( &PhotoNode::triggered_capture, this) );

  }

  ~PhotoNode()
  {
    triggered_thread_.join();

    // shutdown camera
    camera_.photo_camera_close();
  }

  bool setConfig( photo::SetConfig::Request& req, photo::SetConfig::Response& resp )
  {
    photo_mutex_.lock();
    bool error_code = camera_.photo_camera_set_config( req.param, req.value );
    photo_mutex_.unlock();
    return error_code;
  }

  bool getConfig( photo::GetConfig::Request& req, photo::GetConfig::Response& resp )
  {
    char* value = new char[255];
    photo_mutex_.lock();
    bool error_code = camera_.photo_camera_get_config( req.param, &value );
    if( error_code )
    {
      resp.value = value;
    }
    photo_mutex_.unlock();
    delete value;
    return error_code;
  }

  bool capture( photo::Capture::Request& req, photo::Capture::Response& resp )
  {
    // capture a camera image
    photo_mutex_.lock();
    bool error_code = camera_.photo_camera_capture( &image_ );
    if( error_code )
    {
      // fill image message
      fillImage( resp.image, "rgb8", image_.getHeight(), image_.getWidth(), image_.getBytesPerPixel() * image_.getWidth(), image_.getDataAddress() );
    }
    photo_mutex_.unlock();
    return error_code;
  }

  void triggered_capture(void)
  {
    while (ros::ok())
    {
      photo_mutex_.lock();
      bool ret = camera_.triggered_camera_capture( &image_, 10000 );
      if ( ret )
      {
        ROS_INFO("Captured");
        sensor_msgs::Image img;
        fillImage( img, "rgb8", image_.getHeight(), image_.getWidth(), image_.getBytesPerPixel() * image_.getWidth(), image_.getDataAddress() );
        triggered_capture_.publish(img);
      }
      photo_mutex_.unlock();
    }
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "photo");
  PhotoNode a;
  ros::spin();

  return 0;
}
