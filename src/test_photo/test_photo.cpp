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
#include <iostream>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
//#include <opencv_latest/CvBridge.h>
//#include <opencv/highgui.h> //for cvLoadImage()

#include <photo/photo.h>
#include <photo/GetConfig.h>
#include <photo/SetConfig.h>
#include <photo/Capture.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_photo");

//  image_msgs::Image image;
//  image_msgs::CvBridge img_bridge;
//
//  ros::NodeHandle n;
//  ros::ServiceClient client = n.serviceClient<photo::Capture>("/photo/capture");
//  photo::Capture srv;
//  if(client.call(srv))
//  {
//    printf("Calling photo/capture service was successful\n");
//    image = srv.response.image;
//    if (img_bridge.fromImage(image, "bgr"))
//      cvSaveImage("test.jpg", img_bridge.toIpl());
//  }
//  else
//  {
//    printf("Could not query photo/capture service\n");
//  }
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<photo::SetConfig>("/photo/set_config");
  photo::SetConfig srv;
  srv.request.param = "exptime";
  srv.request.value = "20";
  if( !client.call(srv) )
  {
    ROS_FATAL("Could not query set_config service");
  }
  return 0;
}

