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

#include <cstring>
#include <>
#include <gphoto2/gphoto2-port-log.h>

#include "photo/photo_camera.hpp"


photo_camera::photo_camera( void ) :
  camera_(NULL),
  context_(NULL),
  mode_(DIRECT),
  
{
}

photo_camera::~photo_camera( void )
{
  delete camera_;
  delete context_;
}

static void photo_camera::context_error_reporter( GPContext *context, const char *format, va_list args, void *data )
{
  char error_string[1024]; // Maximum size of error message.

  vsnprintf( error_string, 1024, format, args );
  //va_end( args );

  std::cerr << std::endl << "photo_camera: Context error " << std::endl
	    << error_string << std::endl;
}

static void photo_camera::context_status_reporter( GPContext *context, const char *format, va_list args, void *data )
{
  char status_string[1024]; // Maximum size of status message.

  vsnprintf( status_string, 1024, format, args );
  //va_end( args );

  std::cerr << "photo_camera: Status " << status_string << std::endl;
}

static void photo_camera::photo_camera_error_reporter( std::string function_name )
{
  std::cerr << "photo_camera: Error executing function '" << function name << "'." << std::endl;
}

static void photo_camera::photo_camera_error_reporter( std::string function_name, std::string additional_message )
{
  photo_camera_error_reporter( function_name );
  std::cerr << additional_message << std::endl;
}


GPContext* photo_camera::photo_camera_create_context( void )
{
  GPContext* context;

  context = gp_context_new();

  // Optional debugging and status output
  gp_context_set_error_func( context, context_error_reporter, NULL );
  gp_context_set_status_func( context, context_status_reporter, NULL );

  return context;
}


bool photo_camera::photo_camera_load_abilities( GPContext* context )
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

bool photo_camera::photo_camera_lookup_abilities( const std::string model_name )
{
  int list_index = 0;

  // Find the camera in the list of cameras and return the index
  list_index = gp_abilities_list_lookup_model( abilities_list_, model_name.c_str() );
  if( list_index < GP_OK )
  {
    photo_camera_error_reporter( "gp_abilities_list_lookup_model()" );
    return false;
  }

  // Find the camera's abilities in the abilities list
    if( gp_abilities_list_get_abilities( abilities_list_, list_index, &abilities_ ) != GP_OK )
  {
    photo_camera_error_reporter( "gp_abilities_list_get_abilities()" );
    return false;
  }

  return true;
}



bool photo_camera::photo_camera_load_port_info( size_t* port_count )
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


bool photo_camera::photo_camera_lookup_port_info( const std::string port_name )
{
  int list_index = 0;

  // Find the port in the list of ports and return the index
  list_index = gp_port_info_list_lookup_path( port_info_list_, port_name.c_str() );
  if( list_index < GP_OK )
  {
    photo_camera_error_reporter( "gp_port_info_list_lookup_path()" );
    if( list_index == GP_ERROR_UNKNOWN_PORT )
    {
      cerr << "The specified port (" << port << ") cannot be found. Use 'gphoto2 --list-ports' to display available ports. The prefix 'serial:' or 'usb:' is required." << std::endl;
    }
    return false;
  }
  
  // Get the port information from from the information list
  if( gp_port_info_list_get_info( port_info_list, list_index, &port_info_ ) != GP_OK )
  {
    photo_camera_error_reporter( "gp_port_info_list_get_info()" );
    return false;
  }

  return true;
}


bool photo_camera::photo_camera_open( const std::string model_name, const std::string port_name )
{
  // Create a context
  context_ = photo_camera_create_context();

  // Create new camera
  if( gp_camera_new( &camera_ ) != GP_OK )
  {
    photo_camera_error_reporter( "gp_camera_new()" );
    return false;
  }

  // Find and set camera abilities based on model
  if( photo_camera_lookup_abilities( model_name ) == true )
  {
    // Set the camera's abilities
    if( gp_camera_set_abilities( *camera_, abilities_ ) != GP_OK )
    {
      photo_camera_error_reporter( "gp_camera_set_abilities()" );
      return false;
    }
  }
  else
  {
    return false;
  }

  // Associate camera with port
  if( photo_camera_lookup_port_info( port_name ) == true )
  {
    if( gp_camera_set_port_info( *camera_, port_info_ ) != GP_OK )
    {
      photo_camera_error_reporter( "gp_camera_set_port_info()" );
      return false;
    }
  }
  else
  {
    return false;
  }

  // Camera is open!
  return true;
}

bool photo_camera::photo_camera_close( void )
{
  if( gp_camera_exit( camera_, context_ ) != GP_OK )
  {
    photo_camera_error_reporter( "gp_camera_exit()", "Could not close photo_camera.");
    return false;
  }
  return true;
}


/*
 * This function detects all currently attached photo_cameras and returns them
 * in a list. It avoids the generic 'usb:' port entry. This function does not
 * open or initialize the photo_cameras.
 */

bool photo_camera::photo_camera_autodetect( CameraList* camera_list )
{
  CPContext* context;
  int count = 0;
  size_t port_count = 0;
  


  // Create a context
  context = photo_create_context();

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


bool photo_camera::photo_camera_filter_camera_list( CameraList* camera_list, GPContext* context, const std::string match_string )
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




bool photo_camera::photo_camera_open_list( size_t n )
{
  const char *name, *value; 

  gp_list_get_name(camera_list, n, &name);
  gp_list_get_value(camera_list, n, &value);
 
  if( photo_camera_open( name, value ) == false )
  {
    photo_camera_error_reporter( "photo_camera_open()" );
    return false;
  }
  return true;
}

CameraList* photo_camera::photo_camera_get_camera_list( void )
{
  return camera_list_;
}
