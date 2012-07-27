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


#include "photo/photo_camera_list.hpp"


photo_camera_list::photo_camera_list( void ) :
  camera_list_(NULL),
  port_info_list_(NULL),
  abilities_list_(NULL)
{
}

photo_camera_list::~photo_camera_list( void )
{
  delete camera_list_;
  delete port_info_list_;
  delete abilities_list_;
}



CameraList* photo_camera_list::getCameraList( void )
{
  return camera_list_;
}


GPPortInfoList* photo_camera_list::getPortInfoList( void )
{
  return port_info_list_;
}

CameraAbilitiesList* photo_camera_list::getAbilitiesList( void )
{
  return abilities_list_;
}


bool photo_camera_list::loadPortInfo( size_t* port_count )
{
  if( port_info_list_ == NULL )
  {
    // Create a new port info list
    if( gp_port_info_list_new( &port_info_list_ ) != GP_OK )
    {
      photo_camera_error_reporter( "gp_port_info_list_new()" );
      return false;
    }

    // Populate the list
    if( gp_port_info_list_load( port_info_list ) != GP_OK )
    {
      photo_camera_error_reporter( "gp_port_info_list_load()" );
      return false;
    }
  }

  // Count the number of ports in the list
  *port_count =  gp_port_info_list_count( port_info_list_ );
  if( *port_count < GP_OK )
  {
    photo_camera_error_reporter( "gp_port_info_list_count()" );
    return false;
  }

  return true;
}

bool photo_camera_list::loadAbilities( CameraAbilitiesList* abilities, GPContext* context )
{
  // Create a new abilities list
  if( gp_abilities_list_new( &abilities_list_ ) != GP_OK )
  {
    photo_camera_error_reporter( "gp_abilities_list_new()" );
    return false;
  }

  // Populate the abilities list
  if( gp_abilities_list_load( abilities_list_, context ) != GP_OK )
  {
    photo_camera_error_reporter( "gp_abilities_list_load()" );
    return false;
  }

  return true;
}



/*
 * This function detects all currently attached photo_cameras and returns them
 * in a list. It avoids the generic 'usb:' port entry. This function does not
 * open or initialize the photo_cameras.
 */

bool photo_camera_list::autodetect( GPContext* context )
{
  int count = 0;
  size_t port_count = 0;
  
 
  // Create a new list of cameras
  if( gp_list_new( &camera_list ) != GP_OK )
  {
    photo_camera_error_reporter( "gp_list_new()" );
    return false;
  }

  // Load the low-level port drivers
  if( photo_camera_load_port_info( &port_count ) == false )
  {
    return false;
  }

  // Load the photo_camera drivers
  if( photo_camera_load_abilities( context ) == false )
  {
    return false;
  }


  // Filter the list for USB cameras
  if( photo_camera_filter_camera_list( camera_list, context, "usb:" ) == false )
  {
    return false;
  }
  return true;
}



bool photo_camera_list::filterCameraList( GPContext* context, const std::string match_string )
{
  CameraList *working_list = NULL;
  const char *name, *value; 

  if( gp_list_new( &working_list ) != GP_OK )
  {
    photo_camera_error_reporter( "gp_list_new()" );
    gp_list_free( working_list );
    return false;
  }

  // Autodetect the currently attached photo_cameras.
  if( gp_abilities_list_detect( abilities_list_, port_info_list_, working_list, context) != GP_OK )
  {
    photo_camera_error_reporter( "gp_abilities_list_detect()" );
    gp_list_free( working_list );
    return false;
  }
  
 
  count = gp_list_count( working_list );
  if( count < GP_OK )
  {
    photo_camera_error_reporter( "gp_list_count()" );
    gp_list_free( working_list );
    return false;
  }
  
  // Filter out the generic 'usb:' entry
  for( int i = 0; i < count; i++ )
  {

    gp_list_get_name( working_list, i, &name );
    gp_list_get_value( working_list, i, &value );

    if( strcmp( match_string.c_str() , value ) != 0 )
    {
      // This requires camera_list to be empty, but that is not guaranteed
      gp_list_append( camera_list, name, value );
    }
  }
  
  gp_list_free( working_list );

  return true;
}
