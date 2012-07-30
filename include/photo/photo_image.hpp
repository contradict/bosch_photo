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
#ifndef __PHOTO_IMAGE__
#define __PHOTO_IMAGE__


class photo_image
{

private:
  int width_;
  int height_;
  size_t bytes_per_pixel_;
  size_t image_size_;
  char* data_;

public:

  photo_image( void );
  ~photo_image( void );


  int getWidth( void );
  int getHeight( void );
  size_t getBytesPerPixel( void );
  size_t getImageSize( void );
  char* getDataAddress( void );

  //* sets size and allocates memory for image data
  void photo_image_set_size( int image_width, int image_height, size_t image_bytes_per_pixel );

  //* Read an image from filesystem
  /*
   * This function is a debugging function for use replacing photo_image acquisition using a photo_camera.
   * It can only read 24-bit RGB images via OpenCV. It may be extended to handle grayscale and alpha channels in the future.
   */
  bool photo_image_read( std::string filename );

  //* Write a photo_image to filesystem
  /*
   * This function is a debugging function for use replacing photo_image acquisition using a photo_camera.
   * It can only write 24-bit RGB images via OpenCV. It may be extended to handle grayscale and alpha channels in the future.
   */
  bool photo_image_write( std::string filename );
};

#endif // __PHOTO_IMAGE__
