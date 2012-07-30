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

#include <cstdio>  // sscanf, printf, etc.
#include <fcntl.h> // C file I/O
#include <cstdlib>
#include <cstring>

#include "photo/photo_reporter.hpp"

#include "photo/photo_camera.hpp"


photo_camera::photo_camera( void ) :
  camera_(NULL),
  context_(NULL),
  mode_(DIRECT)
{
  //port_info_(NULL);
  //abilities_(NULL);
}

photo_camera::~photo_camera( void )
{
  gp_camera_unref( camera_ ); //delete camera_;
  gp_context_unref( context_ ); //delete context_;
}


GPContext* photo_camera::photo_camera_create_context( void )
{
  context_ = gp_context_new();

  // Optional debugging and status output
  gp_context_set_error_func( context_, photo_reporter::contextError, NULL );
  gp_context_set_status_func( context_, photo_reporter::contextStatus, NULL );
  return context_;
}


bool photo_camera::photo_camera_open( photo_camera_list* list, const std::string model_name, const std::string port_name )
{
  // Create a context if necessary
  if( context_ == NULL )
  {
    context_ = photo_camera_create_context();
  }

  // Create new camera
  if( gp_camera_new( &camera_ ) != GP_OK )
  {
    photo_reporter::error( "gp_camera_new()" );
    return false;
  }

  // Find and set camera abilities based on model
  //std::cout << "Model name: " << model_name << " == " << std::endl;
  if( list->lookupAbilities( model_name, &abilities_ ) == true )
  {
    // Set the camera's abilities
    if( gp_camera_set_abilities( camera_, abilities_ ) != GP_OK )
    {
      photo_reporter::error( "gp_camera_set_abilities()" );
      return false;
    }
  }
  else
  {
    return false;
  }

  // Associate camera with port
  if( list->lookupPortInfo( port_name, &port_info_ ) == true )
  {
    if( gp_camera_set_port_info( camera_, port_info_ ) != GP_OK )
    {
      photo_reporter::error( "gp_camera_set_port_info()" );
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



bool photo_camera::photo_camera_open( photo_camera_list* list, int n )
{
  const char *name, *value; 

  gp_list_get_name( list->getCameraList(), n, &name);
  gp_list_get_value( list->getCameraList(), n, &value);
 
  std::cout << "Opening camera " << n << " by name (" << name << ") and value (" << value << ")" << std::endl;

  if( photo_camera_open( list, name, value ) == false )
  {
    photo_reporter::error( "photo_camera_open()" );
    return false;
  }
  return true;
}


bool photo_camera::photo_camera_close( void )
{
  if( gp_camera_exit( camera_, context_ ) != GP_OK )
  {
    photo_reporter::error( "gp_camera_exit()", "Could not close photo_camera.");
    return false;
  }
  return true;
}




int photo_camera::photo_camera_find_widget_by_name( std::string name, CameraWidget **child, CameraWidget **root)
 {
  int error_code;

  // Get camera configuration
  error_code = gp_camera_get_config( camera_, root, context_ );
  if (error_code != GP_OK)
  {
    photo_reporter::error( "gp_camera_get_config()");
    return error_code;
  }

  // Find child of configuration by name
  if( gp_widget_get_child_by_name( *root, name.c_str(), child ) == GP_OK )
  {
    return GP_OK;
  }

  // Find child of configuration  by label
  if( gp_widget_get_child_by_label( *root, name.c_str(), child ) == GP_OK )
  {
    return GP_OK;
  }

  // If full name is not found, search for last subname.
  // name delimeter is '/'
  size_t found_index = name.length();
  while( found_index == name.length() )
  {
    found_index = name.rfind( '/' );

    if( found_index == std::string::npos ) // No subname, we already failed this search above
    {
      gp_context_error( context_,"%s not found in configuration tree.", name.c_str() );
      gp_widget_free( *root );
      return GP_ERROR;
    }

    if( found_index == name.length() - 1 ) // end of string, cut it off
    {
      name = name.substr( 0, found_index );
    }
  }
  name = name.substr( found_index, name.length() - 1 );

  // Find child using 
  if( gp_widget_get_child_by_name( *root, name.c_str(), child ) == GP_OK )
  {
    return GP_OK;
  }
  if( gp_widget_get_child_by_label( *root, name.c_str(), child ) == GP_OK )
  {
    return GP_OK;
  }

  // all matches have failed
  gp_context_error( context_, "%s not found in configuration tree.", name.c_str() );
  gp_widget_free( *root );
  return GP_ERROR;
}

/** This function should be updated to leverage boost's ability to compare strings
 *  in a case insensitive manner:
 * #include <boost/algorithm/string.hpp>
 * boost::iequals( s1, s2 );
 */
bool photo_camera::photo_camera_check_toggle_value( std::string value_in, bool* value_out )
{
  std::string toggle_positive[] = { "on", "yes", "true", "ON", "YES", "TRUE"};
  std::string toggle_negative[] = { "off", "no", "false", "OFF", "NO", "FALSE" };


  // first check numeric values: "1" and "0"
  if( value_in.compare( "0" ) == 0 )
  {
    *value_out = false;
    return true;
  }
  if( value_in.compare( "0" ) == 0 )
  {
    *value_out = true;
    return true;
  }

  // check values in toggle_positive
  for( int i = 0; i < 6; i++ )
  {
    if( value_in.compare( toggle_positive[i] ) == 0 )
    {
      *value_out = true;
      return true;
    }
  }

  // check values in toggle_negative
  for( int i = 0; i < 6; i++ )
  {
    if( value_in.compare( toggle_negative[i] ) == 0 )
    {
      *value_out = false;
      return true;
    }
  }
  return false;
}



bool photo_camera::photo_camera_set_config( std::string param, std::string value )
{
  CameraWidget *root, *child;
  int error_code;
  const char *label;
  CameraWidgetType type;

  // Locate the widget that corresponds to this parameter
  if( photo_camera_find_widget_by_name( param, &child, &root ) != GP_OK )
  {
    photo_reporter::error( "photo_camera_find_widget_by_name()");
    return false;
  }

  // Get the widget label
  if( gp_widget_get_label(child, &label) != GP_OK )
  {
    photo_reporter::error( "gp_widget_get_label()");
    gp_widget_free( root );
    return false;
  }

  // Get the widget type
  if( gp_widget_get_type( child, &type ) != GP_OK )
  {
    photo_reporter::error( "gp_widget_get_type()");
    gp_widget_free( root );
    return false;
  }
    
  switch( type )
  {

  case GP_WIDGET_TEXT: // char*
    if( gp_widget_set_value(child, value.c_str()) != GP_OK )
    {
      photo_reporter::error( "gp_widget_set_value()");
      gp_context_error( context_, "Failed to set the value of text widget %s to %s.", param.c_str(), value.c_str() );
      gp_widget_free( root );
      return false;
    }
    break;

  case GP_WIDGET_RANGE: // float
    float f, t, b, s;

    if( gp_widget_get_range( child, &b, &t, &s) != GP_OK )
    {
      photo_reporter::error( "gp_widget_get_range()" );
      gp_widget_free( root );
      return false;
    }
    if( !sscanf( value.c_str(), "%f", &f ) )
    {
      gp_context_error( context_, "The passed value %s is not a floating point value.", value.c_str() );
      gp_widget_free( root );
      return false;
    }
    if( (f < b) || (f > t) )
    {
      gp_context_error( context_ , "The passed value %f is not within the expected range of %f -- %f.", f, b, t );
      gp_widget_free( root );
      return false;
    }
    if( gp_widget_set_value( child, &f ) != GP_OK )
    {
      photo_reporter::error( "gp_widget_set_value()" );
      gp_context_error( context_, "Failed to set the value of range widget %s to %f.", param.c_str(), f );
      gp_widget_free( root );
      return false;
    }
    break;

  case GP_WIDGET_TOGGLE: // int
    bool tog;
    if( photo_camera_check_toggle_value( value, &tog ) == false )
    {
      gp_context_error(context_, "The passed value %s is not a valid toggle value.", value.c_str() );
      gp_widget_free( root );
      return false;
    }
    if( gp_widget_set_value( child, &tog ) != GP_OK )
    {
      photo_reporter::error( "gp_widget_set_value()" );
      gp_context_error( context_, "Failed to set values %s of toggle widget %s.", value.c_str(), param.c_str() );
      gp_widget_free( root );
      return false;
    }
    break;
  
  case GP_WIDGET_DATE: // int
  {
    int time = -1;
#ifdef HAVE_STRPTIME
    struct tm xtm;
    
    if( strptime( value.c_str(), "%c", &xtm ) || strptime( value.c_str(), "%Ec", &xtm ) )
    {
      time = mktime( &xtm );
    }
#endif
    if( time == -1 )
    {
      if( !sscanf( value.c_str(), "%d", &time ) )
      {
        gp_context_error( context_, "The passed value %s is neither a valid time nor an integer.", value.c_str() );
	gp_widget_free( root );
	return false;
      }
    }
    if( gp_widget_set_value(child, &time) != GP_OK )
    {
      photo_reporter::error( "gp_widget_set_value()" );
      gp_context_error( context_, "Failed to set new time of date/time widget %s to %s.", param.c_str(), value.c_str() );
      gp_widget_free( root );
      return false;
    }
    break;
  }

  case GP_WIDGET_MENU:
  case GP_WIDGET_RADIO: // char*
    int count, i;
    count = gp_widget_count_choices( child );
    if( count < GP_OK )
    {
      photo_reporter::error( "gp_widget_count_choices()" );
      gp_widget_free( root );
      return false;
    }

    error_code = GP_ERROR_BAD_PARAMETERS;
    for( i = 0; i < count; i++ )
    {
      const char *choice;
      if( gp_widget_get_choice( child, i, &choice ) == GP_OK )
      {
	if( value.compare( choice ) == 0 )
	{
	  if( gp_widget_set_value( child, value.c_str() ) == GP_OK )
	  {
	    break;
	  }
	}
      }
    }
    // attemt a different method for setting a radio button
    if( sscanf( value.c_str(), "%d", &i ) )
    {
      if( (i >= 0) && (i < count) )
      {
        const char *choice;
        if( gp_widget_get_choice( child, i, &choice ) == GP_OK )
	{
	  if( gp_widget_set_value( child, choice ) == GP_OK )
	  {
	    break;
	  }
	}
      }
    }
    gp_context_error( context_, "Choice %s not found within list of choices.", value.c_str() );
    gp_widget_free( root );
    return false;
  
  case GP_WIDGET_WINDOW:
  case GP_WIDGET_SECTION:
  case GP_WIDGET_BUTTON:
  default:
    gp_context_error( context_,"The %s widget is not configurable.", param.c_str() );
    gp_widget_free( root );
    return false;
  }


  // Configuration parameters are correct, so set the camera
  if( gp_camera_set_config( camera_, root, context_ ) != GP_OK )
  {
    photo_reporter::error( "gp_camera_set_config()" );
    gp_context_error( context_, "Failed to set new configuration value %s for configuration entry %s.", value.c_str(), param.c_str() );
    gp_widget_free( root );
    return false;
  }

  gp_widget_free( root );
  return true;
}



bool photo_camera::photo_camera_get_config( std::string param, char** value )
{
  CameraWidget *root, *child;
  const char *label;
  CameraWidgetType type;

  // Locate the widget that corresponds to this parameter
  if( photo_camera_find_widget_by_name( param, &child, &root ) != GP_OK )
  {
    photo_reporter::error( "photo_camera_find_widget_by_name()");
    return false;
  }

  // Get the widget label
  if( gp_widget_get_label(child, &label) != GP_OK )
  {
    photo_reporter::error( "gp_widget_get_label()");
    gp_widget_free( root );
    return false;
  }

  // Get the widget type
  if( gp_widget_get_type( child, &type ) != GP_OK )
  {
    photo_reporter::error( "gp_widget_get_type()");
    gp_widget_free( root );
    return false;
  }

  switch( type )
  {
  case GP_WIDGET_TEXT: // char*
    char *txt;
    if( gp_widget_get_value( child, &txt ) != GP_OK )
    {
      gp_context_error( context_, "Failed to retrieve value of text widget %s.", param.c_str() );
    }
    *value = txt;
    break;
 
  case GP_WIDGET_RANGE: // float
    float f, t,b,s;
    if( gp_widget_get_range( child, &b, &t, &s ) != GP_OK )
    {
      gp_context_error( context_, "Failed to retrieve values of range widget %s.", param.c_str() );
    }
    if( gp_widget_get_value( child, &f ) != GP_OK )
    {
      gp_context_error( context_, "Failed to value of range widget %s.", param.c_str() );
    }
    sprintf( *value, "%f", f );
    break;

  case GP_WIDGET_TOGGLE: // int
  {
    int t;
    if( gp_widget_get_value( child, &t ) != GP_OK )
    {
      gp_context_error( context_,"Failed to retrieve values of toggle widget %s.", param.c_str() );
    }
    sprintf( *value, "%d", t );
    break;
  }

  case GP_WIDGET_DATE: // int
  {
    int error_code, t;
    time_t working_time;
    struct tm *localtm;
    char timebuf[200];

    if( gp_widget_get_value( child, &t ) != GP_OK )
    {
      gp_context_error( context_,"Failed to retrieve values of date/time widget %s.", param.c_str() );
      break;
    }
    working_time = t;
    localtm = localtime( &working_time );
    error_code = strftime( timebuf, sizeof(timebuf), "%c", localtm );
    sprintf( *value, "%s", timebuf );
    break;
  }

  case GP_WIDGET_MENU:
  case GP_WIDGET_RADIO: //char*
    char *current;
    if( gp_widget_get_value (child, &current) != GP_OK )
    {
      gp_context_error( context_,"Failed to retrieve values of radio widget %s.", param.c_str() );
    }
    sprintf( *value, "%s", current );
    break;

  // No values, so nothing to return
  case GP_WIDGET_WINDOW:
  case GP_WIDGET_SECTION:
  case GP_WIDGET_BUTTON:
  default:
    break;
  }

  gp_widget_free( root );
  return true;
}


bool photo_camera::photo_camera_capture_to_file( std::string filename )
{
  int fd, error_code;
  CameraFile *photo_file;
  CameraFilePath photo_file_path;

  // NOP: This gets overridden in the library to /capt0000.jpg
  strcpy( photo_file_path.folder, "/");
  strcpy( photo_file_path.name, "foo.jpg");

  error_code = gp_camera_capture( camera_, GP_CAPTURE_IMAGE, &photo_file_path, context_ );
  if( error_code < GP_OK )
  {
    photo_reporter::error( "gp_camera_capture()" );
    gp_context_error( context_, "Could not capture image  (error code %d)\n", error_code );
    return false;
  }

  fd = open( filename.c_str(), O_CREAT|O_WRONLY, 0644 );
  error_code = gp_file_new_from_fd( &photo_file, fd );
  if( error_code < GP_OK )
  {
    photo_reporter::error( "gp_file_new_from_fd()" );
    gp_context_error( context_, "Could not create a new image file from %s%s (error code %d)\n", photo_file_path.folder, photo_file_path.name, error_code );
    gp_file_free( photo_file );
    return false;
  }

  error_code = gp_camera_file_get( camera_, photo_file_path.folder, photo_file_path.name, GP_FILE_TYPE_NORMAL, photo_file, context_ );
  if( error_code < GP_OK )
  {
    photo_reporter::error( "gp_camera_file_get()" );
    gp_context_error( context_, "Could not get file %s%s (error code %d)\n", photo_file_path.folder, photo_file_path.name, error_code );
    gp_file_free( photo_file );
    return false;
  }

  error_code = gp_camera_file_delete( camera_, photo_file_path.folder, photo_file_path.name, context_ );
  if( error_code < GP_OK )
  {
    photo_reporter::error( "gp_camera_file_delete()" );
    gp_context_error( context_, "Could delete file %s%s  (error code %d)\n", photo_file_path.folder, photo_file_path.name, error_code );
    gp_file_free( photo_file );
    return false;
  }

  gp_file_free( photo_file );
  return true;
}


bool photo_camera::photo_camera_capture( photo_image* image )
{
  int fd, error_code;
  CameraFile *photo_file;
  CameraFilePath photo_file_path;
  char temp_file_name[20];

  // NOP: This gets overridden in the library to /capt0000.jpg
  strcpy( photo_file_path.folder, "/" );
  strcpy( photo_file_path.name, "foo.jpg" );

  error_code = gp_camera_capture( camera_, GP_CAPTURE_IMAGE, &photo_file_path, context_ );
  if( error_code < GP_OK )
  {
    photo_reporter::error( "gp_camera_capture()" );
    gp_context_error( context_, "Could not capture image  (error code %d)\n", error_code );
    return false;
  }

  // create temporary file
  strcpy( temp_file_name, "tmpfileXXXXXX" );
  fd = mkstemp( temp_file_name );
  error_code = gp_file_new_from_fd( &photo_file, fd );
  if( error_code < GP_OK )
  {
    close( fd );
    unlink( temp_file_name );

    photo_reporter::error( "gp_file_new_from_fd()" );
    gp_context_error( context_, "Could not create a new image file from %s%s (error code %d)\n", photo_file_path.folder, photo_file_path.name, error_code );
    gp_file_free( photo_file );
    return false;
  }

  // get image from camera and store in temporary file
  error_code = gp_camera_file_get( camera_, photo_file_path.folder, photo_file_path.name, GP_FILE_TYPE_NORMAL, photo_file, context_ );
  if( error_code < GP_OK )
  {
    gp_file_unref( photo_file );
    unlink( temp_file_name );
    photo_reporter::error( "gp_camera_file_get()" );
    gp_context_error( context_, "Could not get file %s%s (error code %d)\n", photo_file_path.folder, photo_file_path.name, error_code );
    return false;
  }

  // delete image from camera's memory
  error_code = gp_camera_file_delete( camera_, photo_file_path.folder, photo_file_path.name, context_ );
  if( error_code < GP_OK )
  {
    unlink( temp_file_name );
    photo_reporter::error( "gp_camera_file_delete()" );
    gp_context_error( context_, "Could delete file %s%s  (error code %d)\n", photo_file_path.folder, photo_file_path.name, error_code );
    gp_file_free( photo_file );
    return false;
  }

  // load image from temporary file
  if( image->photo_image_read( std::string(temp_file_name) ) == true )
  {
    gp_file_free( photo_file );
    unlink( temp_file_name );
    return true;
  }

  photo_reporter::error( "photo_image_read()" );
  gp_file_free( photo_file );
  unlink( temp_file_name );
  return false;
}



