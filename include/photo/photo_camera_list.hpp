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
#ifndef __PHOTO_CAMERA_LIST__
#define __PHOTO_CAMERA_LIST__

#include <string>

#include <gphoto2/gphoto2-camera.h>
#include <gphoto2/gphoto2-setting.h>
#include <gphoto2/gphoto2-filesys.h>

class photo_camera_list
{

private:
  CameraList* camera_list_;
  GPPortInfoList* port_info_list_;
  CameraAbilitiesList* abilities_list_;

public:
  photo_camera_list( void );
  ~photo_camera_list( void );

  CameraList* getCameraList( void );
  GPPortInfoList* getPortInfoList( void );
  CameraAbilitiesList* getAbilitiesList( void );

  bool filterCameraList( GPContext* context, const std::string match_string );


  //* Autodetect all photo_cameras connected to the system
  bool autodetect( GPContext* context );

  bool loadPortInfo( ssize_t* port_count );
  bool loadAbilities( GPContext* context );

  //* Look up the port information for the port 'port_name'
  bool lookupPortInfo( const std::string port_name, GPPortInfo* port_info );
  //* Look up abilities for the camera model 'model_name'
  bool lookupAbilities( const std::string model_name, CameraAbilities* abilities );



};

#endif // __PHOTO_CAMERA_LIST__
