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
#ifndef PHOTO_H
#define PHOTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <gphoto2/gphoto2-camera.h>
#include <gphoto2/gphoto2-port-log.h>
#include <gphoto2/gphoto2-setting.h>
#include <gphoto2/gphoto2-filesys.h>
#include <photo/photo_image.h>

#define PHOTO_MODE_DIRECT     0
#define PHOTO_MODE_TO_FILE    1

typedef struct {
  Camera *cam;
  GPContext *context;
  int mode;
} photo_t, *photo_p;

/* initialize a new photo */
photo_p photo_initialize(void);

/* release photo memory */
void photo_free(photo_p photo);

/* open a specific USB connection the photo */
int photo_open(photo_p photo, const char *model, const char *port);

/* open the first photo detected on the USB bus */
int photo_autodetect(photo_p photo);

/* close the photo */
int photo_close(photo_p photo);

/* set a photo parameter */
int photo_set_config(photo_p photo, const char *param, const char *value);

/* get a photo parameter */
int photo_get_config(photo_p photo, const char *param, char **value);

/* capture an image */
int photo_capture(photo_p photo, photo_image_p image);

/* capture an image to file */
int photo_capture_to_file(photo_p photo, const char *filename);

#ifdef __cplusplus
}
#endif

#endif
