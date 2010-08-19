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
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <photo/photo.h>
#include <photo/photo_image.h>

int photo_capture_to_file(photo_p photo, const char *filename)
{
  int fd, ret;
  CameraFile *photofile;
  CameraFilePath photo_file_path;

  /* NOP: This gets overridden in the library to /capt0000.jpg */
  strcpy(photo_file_path.folder, "/");
  strcpy(photo_file_path.name, "foo.jpg");

  ret = gp_camera_capture(photo->cam, GP_CAPTURE_IMAGE, &photo_file_path, photo->context);
  if (ret<GP_OK) {
    fprintf(stderr, "Could not capture image  (error code %d)\n", ret);
    return ret;
  }

  fd = open(filename, O_CREAT|O_WRONLY, 0644);
  ret = gp_file_new_from_fd(&photofile, fd);
  if (ret<GP_OK) {
    fprintf(stderr, "Could not create a new image file from %s%s (error code %d)\n", photo_file_path.folder,photo_file_path.name,ret);
    goto out;
  }

  ret = gp_camera_file_get(photo->cam, photo_file_path.folder, photo_file_path.name, GP_FILE_TYPE_NORMAL, photofile, photo->context);
  if (ret<GP_OK) {
    fprintf(stderr, "Could not get file %s%s (error code %d)\n", photo_file_path.folder,photo_file_path.name,ret);
//    goto out;
  }

  ret = gp_camera_file_delete(photo->cam, photo_file_path.folder, photo_file_path.name, photo->context);
  if (ret<GP_OK) {
    fprintf(stderr, "Could delete file %s%s  (error code %d)\n", photo_file_path.folder,photo_file_path.name,ret);
    goto out;
  }

  out: gp_file_free(photofile);
  return (ret==GP_OK);
}


int photo_capture(photo_p photo, photo_image_p image)
{
  int fd, ret;
  CameraFile *photofile;
  CameraFilePath photo_file_path;
  char  tmpname[20];

  /* NOP: This gets overridden in the library to /capt0000.jpg */
  strcpy(photo_file_path.folder, "/");
  strcpy(photo_file_path.name, "foo.jpg");

  ret = gp_camera_capture(photo->cam, GP_CAPTURE_IMAGE, &photo_file_path, photo->context);
  if (ret<GP_OK) {
    fprintf(stderr, "Could not capture image  (error code %d)\n", ret);
    return ret;
  }

  /* create temporary file */
  strcpy (tmpname, "tmpfileXXXXXX");
  fd = mkstemp(tmpname);
  ret = gp_file_new_from_fd (&photofile, fd);
  if (ret < GP_OK) {
    close (fd);
    unlink(tmpname);
    return ret;
  }

  /* get image from camera */
  ret = gp_camera_file_get (photo->cam, photo_file_path.folder, photo_file_path.name, GP_FILE_TYPE_NORMAL, photofile, photo->context);
  if (ret < GP_OK) {
    gp_file_unref(photofile);
    unlink(tmpname);
    return ret;
  }

  ret = gp_camera_file_delete(photo->cam, photo_file_path.folder, photo_file_path.name, photo->context);
  if (ret<GP_OK) {
    fprintf(stderr, "Could delete file %s%s  (error code %d)\n", photo_file_path.folder,photo_file_path.name,ret);
    goto out;
  }

  if(photo_image_read(image,tmpname))
    ret=GP_OK;
  else
    ret=-1;

  out:
  gp_file_free(photofile);
  unlink(tmpname);
  return (ret==GP_OK);
}



