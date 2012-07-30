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

#include <iostream>

#include "photo/photo_reporter.hpp"

#include "photo/photo_camera_list.hpp"


photo_camera_list::photo_camera_list( void ) :
  camera_list_(NULL),
  port_info_list_(NULL),
  abilities_list_(NULL)
{
}

photo_camera_list::~photo_camera_list( void )
{
  gp_list_unref( camera_list_ ); //delete camera_list_;
  gp_port_info_list_free( port_info_list_ ); //delete port_info_list_;
  gp_abilities_list_free( abilities_list_ );  //delete abilities_list_;
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


bool photo_camera_list::loadPortInfo( ssize_t* port_count )
{
  if( port_info_list_ == NULL )
  {
    // Create a new port info list
    if( gp_port_info_list_new( &port_info_list_ ) != GP_OK )
    {
      photo_reporter::error( "gp_port_info_list_new()" );
      return false;
    }

    // Populate the list
    if( gp_port_info_list_load( port_info_list_ ) != GP_OK )
    {
      photo_reporter::error( "gp_port_info_list_load()" );
      return false;
    }
  }

  // Count the number of ports in the list
  *port_count =  gp_port_info_list_count( port_info_list_ );
  if( *port_count < GP_OK )
  {
    photo_reporter::error( "gp_port_info_list_count()" );
    return false;
  }

  return true;
}

bool photo_camera_list::loadAbilities( GPContext* context )
{
  // Create a new abilities list
  if( gp_abilities_list_new( &abilities_list_ ) != GP_OK )
  {
    photo_reporter::error( "gp_abilities_list_new()" );
    return false;
  }

  // Populate the abilities list
  if( gp_abilities_list_load( abilities_list_, context ) != GP_OK )
  {
    photo_reporter::error( "gp_abilities_list_load()" );
    return false;
  }

  return true;
}



bool photo_camera_list::lookupPortInfo( const std::string port_name, GPPortInfo* port_info )
{
  int list_index = 0;

  // Find the port in the list of ports and return the index
  list_index = gp_port_info_list_lookup_path( port_info_list_, port_name.c_str() );
  if( list_index < GP_OK )
  {
    photo_reporter::error( "gp_port_info_list_lookup_path()" );
    if( list_index == GP_ERROR_UNKNOWN_PORT )
    {
      std::cerr << "The specified port (" << port_name << ") cannot be found. Use 'gphoto2 --list-ports' to display available ports. The prefix 'serial:' or 'usb:' is required." << std::endl;
    }
    return false;
  }
  
  // Get the port information from from the information list
  if( gp_port_info_list_get_info( port_info_list_, list_index, port_info ) != GP_OK )
  {
    photo_reporter::error( "gp_port_info_list_get_info()" );
    return false;
  }

  return true;
}


bool photo_camera_list::lookupAbilities( const std::string model_name, CameraAbilities* abilities )
{
  int list_index = 0;

  // Find the camera in the list of cameras and return the index
  list_index = gp_abilities_list_lookup_model( abilities_list_, model_name.c_str() );
  if( list_index < GP_OK )
  {
    photo_reporter::error( "gp_abilities_list_lookup_model()" );
    return false;
  }

  // Find the camera's abilities in the abilities list
  if( gp_abilities_list_get_abilities( abilities_list_, list_index, abilities ) != GP_OK )
  {
    photo_reporter::error( "gp_abilities_list_get_abilities()" );
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
  ssize_t port_count = 0;
 
  // Create a new list of cameras
  if( gp_list_new( &camera_list_ ) != GP_OK )
  {
    photo_reporter::error( "gp_list_new()" );
    return false;
  }

  // Load the low-level port drivers
  if( loadPortInfo( &port_count ) == false )
  {
    return false;
  }

  // Load the photo_camera drivers
  if( loadAbilities( context ) == false )
  {
    return false;
  }


  // Filter the list for USB cameras
  if( filterCameraList( context, "usb:" ) == false )
  {
    return false;
  }
  return true;
}



bool photo_camera_list::filterCameraList( GPContext* context, const std::string match_string )
{
  CameraList *working_list = NULL;
  const char *name, *value;
  int count = 0;

  if( gp_list_new( &working_list ) != GP_OK )
  {
    photo_reporter::error( "gp_list_new()" );
    gp_list_free( working_list );
    return false;
  }

  // Autodetect the currently attached photo_cameras.
  if( gp_abilities_list_detect( abilities_list_, port_info_list_, working_list, context) != GP_OK )
  {
    photo_reporter::error( "gp_abilities_list_detect()" );
    gp_list_free( working_list );
    return false;
  }
  
 
  count = gp_list_count( working_list );
  if( count < GP_OK )
  {
    photo_reporter::error( "gp_list_count()" );
    gp_list_free( working_list );
    return false;
  }
  
  // Clear camera_list_ for appending
  if( gp_list_reset( camera_list_ ) != GP_OK )
  {
    photo_reporter::error( "gp_list_reset()" );
    gp_list_free( working_list );
    return false;
  }

  // Filter out the generic 'usb:' entry
  for( int i = 0; i < count; i++ )
  {

    gp_list_get_name( working_list, i, &name );
    gp_list_get_value( working_list, i, &value );

    if( match_string.compare( value ) != 0 )
    {
      gp_list_append( camera_list_, name, value );
    }
  }
  
  gp_list_free( working_list );

  return true;
}

