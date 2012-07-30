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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

#include <photo/photo_image.hpp>

photo_image::photo_image( void ) :
  width_(0),
  height_(0),
  bytes_per_pixel_(3),
  image_size_(0),
  data_(NULL)
{
}

photo_image::~photo_image( void )
{
  delete[] data_; // delete on a NULL pointer has no effect, so this will work if data_ points to an image or not
}


int photo_image::getWidth( void )
{
  return width_;
}

int photo_image::getHeight( void )
{
  return height_;
}

size_t photo_image::getBytesPerPixel( void )
{
  return bytes_per_pixel_;
}

size_t photo_image::getImageSize( void )
{
  return image_size_;
}

char* photo_image::getDataAddress( void )
{
  return data_;
}

//* sets size and allocates memory for image data
void photo_image::photo_image_set_size( int image_width, int image_height, size_t image_bytes_per_pixel )
{
  delete[] data_; // delete on a NULL pointer has no effect

  width_ = image_width;
  height_ = image_height;
  bytes_per_pixel_ = image_bytes_per_pixel;
  image_size_ = width_ * height_ * bytes_per_pixel_;
  
  data_ = new char[image_size_](); // create array and initialize to default value: 0
}

//* reads an image from filesystem
bool photo_image::photo_image_read( std::string filename )
{
  int r, c;
  // Read image from file using OpenCV
  cv::Mat img = cv::imread( filename.c_str() );
  if( img.empty() )
  {
    std::cerr << "img.empty() == true" << std::endl;
    return false;
  }

  int w = img.cols;
  int h = img.rows;


  //int chan = img.channels(); // number of data channels, ex: 3 for RGB
  int d = img.elemSize(); // bytes per element: 1 element has 'chan' channels of data

  // Store image in photo_image
  if( width_ != w || height_ != h )
  {
    //std::cout << "Setting size to " << w << " x " << h << " x " << d << ".";
    photo_image_set_size( w, h, d );
  }

  size_t n = 0; // counter for iterating over data_
  for( r = 0; r < height_; ++r )
  {
    for( c = 0; c < width_; ++c )
    {
      // This assumes the image is 24-bit RGB. Needs improvement to handle other image types.
      const cv::Vec3b& pixel = img.at<cv::Vec3b>(r, c);

      uint8_t R = pixel[2]; // Red
      uint8_t G = pixel[1]; // Green
      uint8_t B = pixel[0]; // Blue

      data_[n++] = R; // R
      data_[n++] = G; // G
      data_[n++] = B; // B
    }
  }
  return true;
}

//* writes an image to filesystem
bool photo_image::photo_image_write( std::string filename )
{
  int r, c;

  // Create OpenCV image for 24-bit RGB
  cv::Mat img (height_, width_, CV_8UC3);

  int n = 0;
  for( r = 0; r < height_; ++r )
  {
    for( c = 0; c < width_; ++c )
    {
      img.at<unsigned char> (r, 3*c+2) = data_[n++]; // Red
      img.at<unsigned char> (r, 3*c+1) = data_[n++]; // Green
      img.at<unsigned char> (r, 3*c+0) = data_[n++]; // Blue
    }
  }
  cv::imwrite( filename.c_str(), img );
  return true;
}
