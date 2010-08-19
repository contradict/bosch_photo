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
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <photo/photo.h>

static void
ctx_error_func (GPContext *context, const char *format, va_list args, void *data)
{
  fprintf  (stderr, "\n");
  fprintf  (stderr, "*** Contexterror ***              \n");
  vfprintf (stderr, format, args);
  fprintf  (stderr, "\n");
  fflush   (stderr);
}

static void
ctx_status_func (GPContext *context, const char *format, va_list args, void *data)
{
  vfprintf (stderr, format, args);
  fprintf  (stderr, "\n");
  fflush   (stderr);
}

static GPPortInfoList *portinfolist = NULL;
static CameraAbilitiesList *abilities = NULL;

/*
 * This detects all currently attached photos and returns
 * them in a list. It avoids the generic usb: entry.
 *
 * This function does not open nor initialize the photos yet.
 */
int camera_autodetect(CameraList *list, GPContext *context)
{
  int ret, i;
  CameraList *xlist = NULL;

  ret = gp_list_new(&xlist);
  if (ret<GP_OK)
    goto out;
  if (!portinfolist) {
    /* Load all the port drivers we have... */
    ret = gp_port_info_list_new(&portinfolist);
    if (ret<GP_OK)
      goto out;
    ret = gp_port_info_list_load(portinfolist);
    if (ret<0)
      goto out;
    ret = gp_port_info_list_count(portinfolist);
    if (ret<0)
      goto out;
  }
  /* Load all the photo drivers we have... */
  ret = gp_abilities_list_new(&abilities);
  if (ret<GP_OK)
    goto out;
  ret = gp_abilities_list_load(abilities, context);
  if (ret<GP_OK)
    goto out;

  /* ... and autodetect the currently attached photos. */
  ret = gp_abilities_list_detect(abilities, portinfolist, xlist, context);
  if (ret<GP_OK)
    goto out;

  /* Filter out the "usb:" entry */
  ret = gp_list_count(xlist);
  if (ret<GP_OK)
    goto out;
  for(i = 0; i<ret; i++) {
    const char *name, *value;
    gp_list_get_name(xlist, i, &name);
    gp_list_get_value(xlist, i, &value);
    if (!strcmp("usb:", value))
      continue;
    gp_list_append(list, name, value);
  }
  out: gp_list_free(xlist);
  return gp_list_count(list);
}

/*
 * This function opens a camera depending on the specified model and port.
 */
static int camera_open(Camera ** photo, const char *model, const char *port)
{
  int ret, m, p;
  CameraAbilities a;
  GPPortInfo pi;

  ret = gp_camera_new(photo);
  if (ret<GP_OK)
    return ret;

  /* First lookup the model / driver */
  m = gp_abilities_list_lookup_model(abilities, model);
  if (m<GP_OK)
    return ret;
  ret = gp_abilities_list_get_abilities(abilities, m, &a);
  if (ret<GP_OK)
    return ret;
  ret = gp_camera_set_abilities(*photo, a);
  if (ret<GP_OK)
    return ret;

  /* Then associate the photo with the specified port */
  p = gp_port_info_list_lookup_path(portinfolist, port);
  if (ret<GP_OK)
    return ret;
  switch (p) {
  case GP_ERROR_UNKNOWN_PORT:
    fprintf(stderr, "The port you specified "
      "('%s') can not be found. Please "
      "specify one of the ports found by "
      "'gphoto2 --list-ports' and make "
      "sure the spelling is correct "
      "(i.e. with prefix 'serial:' or 'usb:').", port);
    break;
  default:
    break;
  }
  if (ret<GP_OK)
    return ret;
  ret = gp_port_info_list_get_info(portinfolist, p, &pi);
  if (ret<GP_OK)
    return ret;
  ret = gp_camera_set_port_info(*photo, pi);
  if (ret<GP_OK)
    return ret;
  return GP_OK;
}

GPContext* photo_create_context()
{
  GPContext *context;

  /* This is the mandatory part */
  context = gp_context_new();

  /* All the parts below are optional! */
  gp_context_set_error_func (context, ctx_error_func, NULL);
  gp_context_set_status_func (context, ctx_status_func, NULL);

  return context;
}

photo_p photo_initialize(void)
{
  photo_p photo;
  photo = (photo_p)calloc(1, sizeof(photo_t));
  photo->context = NULL;
  photo->cam = NULL;
  return photo;
}

void photo_free(photo_p photo)
{
  if(photo->context)
    free(photo->context);
  if(photo->cam)
     free(photo->cam);
  free(photo);
}

int photo_open(photo_p photo, const char *model, const char *port)
{
  int ret;

  /* create a context */
  photo->context = photo_create_context();

  /* open a specific photo on port */
  ret = camera_open(&photo->cam, model, port);
  if (ret != GP_OK) {
    fprintf(stderr, "Camera %s on port %s failed to open\n", model, port);
  }

  return (ret == GP_OK);
}

int photo_autodetect(photo_p photo)
{
  CameraList *list;
  Camera **cams;
  int ret, i, count;
  const char *name, *value;
  int selected = -1;

  /* create a context */
  photo->context = photo_create_context();

  /* Detect all the photos that can be autodetected... */
  ret = gp_list_new(&list);
  if (ret<GP_OK)
    return 0;
  count = camera_autodetect(list, photo->context);

  /* Now open all photos we autodected for usage */
  printf("Number of photos: %d\n", count);
  cams = (Camera**)calloc(sizeof(Camera*), count);
  for(i = 0; i<count; i++) {
    gp_list_get_name(list, i, &name);
    gp_list_get_value(list, i, &value);
    ret = camera_open(&cams[i], name, value);
    if (ret==GP_OK) {
      selected = i;
      photo->cam = cams[i];
      break;
    }
    else {
      fprintf(stderr, "Camera %s on port %s failed to open\n", name, value);
    }
  }

  if(selected<0) {
    fprintf(stderr, "Could not find any photo.\n");
  }

  return (selected>=0);
}


//int photo_autodetect(photo_p photo)
//{
//  int ret; //, i, count;
//  CameraText text;
//
//  /* create a context */
//  photo->context = photo_create_context();
//
//  /* create a new camera */
//  gp_camera_new(&photo->cam);
//
//  /* this will autodetect the cameras, and take the first one */
//  ret = gp_camera_init(photo->cam,photo->context);
//  if (ret < GP_OK) {
//    printf("No camera auto detected.\n");
//    gp_camera_free(photo->cam);
//    return 0;
//  }
//
//  /* get camera summary */
//  ret = gp_camera_get_summary(photo->cam,&text,photo->context);
//  if (ret < GP_OK) {
//    printf("Camera failed retrieving summary.\n");
//    gp_camera_free(photo->cam);
//    return 0;
//  }
//
//  printf("Summary:\n%s\n",text.text);
//  return 1;
//}

int photo_close(photo_p photo)
{
  int ret;
  ret = gp_camera_exit(photo->cam, photo->context);
  if (ret != GP_OK) {
    fprintf(stderr, "Could not close photo\n");
  }
  return (ret == GP_OK);
}




