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

//#include <string>

#include <gphoto2/gphoto2-camera.h>
#include <gphoto2/gphoto2-context.h>
#include <gphoto2/gphoto2-setting.h>
#include <gphoto2/gphoto2-filesys.h>


#include "photo/photo_camera_list.hpp"
#include "photo/photo_image.hpp"

class photo_camera
{

public:
  enum photo_mode { DIRECT = 0, TO_FILE = 1 };

private:
  Camera* camera_;
  GPContext *context_;
  GPPortInfo port_info_;
  CameraAbilities abilities_;
  enum photo_mode mode_;

  
public:
  
  photo_camera( void );
  ~photo_camera( void );

  //* Create a gphoto_camera context and set reporting functions
  GPContext* photo_camera_create_context( void );


  //* Open the 'n'th photo_camera in the list
  bool photo_camera_open( photo_camera_list* list, int n );

  //* Open a connection the photo_camera of 'model' on port 'port'.
  bool photo_camera_open( photo_camera_list* list, const std::string model_name, std::string port_name );
  
  //* Close the photo_camera
  bool photo_camera_close( void );
  

  //* set a photo_camera parameter
  bool photo_camera_set_config( std::string param, std::string value );
  
  //* get a photo_camera parameter
  bool photo_camera_get_config( std::string, char** value);
  
  //* capture an image
  bool photo_camera_capture( photo_image* image );
  
  //* capture an image to file
  bool photo_camera_capture_to_file( std::string filename );



  int photo_camera_find_widget_by_name( std::string param, CameraWidget **child, CameraWidget **rootconfig );

private:
  bool photo_camera_check_toggle_value( std::string value_in, bool* value_out );
};



#endif //__PHOTO_CAMERA__
