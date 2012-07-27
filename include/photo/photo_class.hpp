/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Robert Bosch LLC.
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
#ifndef __PHOTO_CAMERA__
#define __PHOTO_CAMERA__


#include <gphoto2/gphoto2-camera.h>
#include <gphoto2/gphoto2-setting.h>
#include <gphoto2/gphoto2-filesys.h>


class photo_camera
{

public:
  enum photo_mode = { DIRECT = 0, TO_FILE = 1 }

private:
  enum photo_mode mode_;
  Camera* camera_;
  CameraList* camera_list_;
  CameraAbilities abilities_;
  CameraAbilitiesList* abilities_list_;
  GPContext *context_;
  GPPortInfo port_info_;
  GPPortInfoList* port_info_list_;
  
public:
  
  photo( void );
  ~photo( void );

  CameraList* photo_camera_get_camera_list( void );

  //* Autodetect all photo_cameras connected to the system
  bool photo_camera_autodetect( CameraList* camera_list );

  //* Open the 'n'th photo_camera in the list
  bool photo_camera_open_list( CameraList* camera_list, size_t n );

  //* Open a connection the photo_camera of 'model' on port 'port'.
  bool photo_camera_open( const std::string model, std::string port );
  
  //* Close the photo_camera
  bool photo_camera_close( void );
  
  //* set a photo_camera parameter
  int photo_camera_set_config(photo_camera_p photo_camera, const char *param, const char *value);
  
  //* get a photo_camera parameter
  int photo_camera_get_config(photo_camera_p photo_camera, const char *param, char **value);
  
  //* capture an image
  int photo_camera_capture(photo_camera_p photo_camera, photo_camera_image_p image);
  
  //* capture an image to file
  int photo_camera_capture_to_file(photo_camera_p photo_camera, const char *filename);

private:

  static void context_error_reporter( GPContext* context, const char* format, va_list args, void* data );
  static void context_status_reporter( GPContext* context, const char* format, va_list args, void* data );
  static void photo_camera_error_reporter( std::string function_name );
  static void photo_camera_error_reporter( std::string function_name, std::string additional_message );

  //* Create a gphoto_camera context and set reporting functions
  GPContext* photo_camera_create_context( void );

  bool photo_camera_load_abilities( GPContext* context );
  //* Look up abilities for the camera model 'model_name'
  bool photo_camera_lookup_abilities( const std::string model_name );

  bool photo_camera_load_port_info( size_t* port_count );
  //* Look up the port information for the port 'port_name'
  bool photo_camera_lookup_port_info( const std::string port_name );

  bool photo_camera_filter_camera_list( CameraList* camera_list, GPContext* context, const std::string match_string );
};


#endif //__PHOTO_CAMERA__
