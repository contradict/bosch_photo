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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <photo/photo.h>

static int
find_widget_by_name (Camera *photo, GPContext *context, const char *param, CameraWidget **child, CameraWidget **rootconfig) {
  int ret;

  ret = gp_camera_get_config (photo, rootconfig, context);
  if (ret != GP_OK) return ret;
  ret = gp_widget_get_child_by_name (*rootconfig, param, child);
  if (ret != GP_OK)
    ret = gp_widget_get_child_by_label (*rootconfig, param, child);
  if (ret != GP_OK) {
    char    *part, *s, *newname;

    newname = strdup (param);
    if (!newname)
      return GP_ERROR_NO_MEMORY;

    *child = *rootconfig;
    part = newname;
    while (part[0] == '/')
      part++;
    while (1) {
      CameraWidget *tmp;

      s = strchr (part,'/');
      if (s)
        *s='\0';
      ret = gp_widget_get_child_by_name (*child, part, &tmp);
      if (ret != GP_OK)
        ret = gp_widget_get_child_by_label (*child, part, &tmp);
      if (ret != GP_OK)
        break;
      *child = tmp;
      if (!s) /* end of path */
        break;
      part = s+1;
      while (part[0] == '/')
        part++;
    }
    if (s) { /* if we have stuff left over, we failed */
      gp_context_error (context,"%s not found in configuration tree.", newname);
      free (newname);
      gp_widget_free (*rootconfig);
      return GP_ERROR;
    }
    free (newname);
  }
  return GP_OK;
}

int photo_set_config(photo_p photo, const char *param, const char *value)
{
  CameraWidget *rootconfig, *child;
  int ret;
  const char *label;
  CameraWidgetType type;

  ret = find_widget_by_name(photo->cam, photo->context, param, &child, &rootconfig);
  if (ret!=GP_OK)
    return 0;

  ret = gp_widget_get_type(child, &type);
  if (ret!=GP_OK) {
    gp_widget_free(rootconfig);
    return 0;
  }
  ret = gp_widget_get_label(child, &label);
  if (ret!=GP_OK) {
    gp_widget_free(rootconfig);
    return 0;
  }

  switch (type) {
  case GP_WIDGET_TEXT: { /* char *   */
    ret = gp_widget_set_value(child, value);
    if (ret!=GP_OK)
      gp_context_error(photo->context,"Failed to set the value of text widget %s to %s.", param, value);
    break;
  }
  case GP_WIDGET_RANGE: { /* float    */
    float f, t, b, s;

    ret = gp_widget_get_range(child, &b, &t, &s);
    if (ret!=GP_OK)
      break;
    if (!sscanf(value, "%f", &f)) {
      gp_context_error(photo->context,"The passed value %s is not a floating point value.", value);
      ret = GP_ERROR_BAD_PARAMETERS;
      break;
    }
    if ((f<b)||(f>t)) {
      gp_context_error(photo->context,"The passed value %f is not within the expected range %f - %f.", f, b, t);
      ret = GP_ERROR_BAD_PARAMETERS;
      break;
    }
    ret = gp_widget_set_value(child, &f);
    if (ret!=GP_OK)
      gp_context_error(photo->context,"Failed to set the value of range widget %s to %f.", param, f);
    break;
  }
  case GP_WIDGET_TOGGLE: { /* int    */
    int t;

    t = 2;
    if (!strcasecmp(value, "off")||
        !strcasecmp(value, "no")||
        !strcasecmp(value, "false")||
        !strcmp(value, "0"))
      t = 0;
    if (!strcasecmp(value, "on")||
        !strcasecmp(value, "yes")||
        !strcasecmp(value, "true")||
        !strcmp(value, "1"))
      t = 1;
    /*fprintf (stderr," value %s, t %d\n", value, t);*/
    if (t==2) {
      gp_context_error(photo->context,"The passed value %s is not a valid toggle value.", value);
      ret = GP_ERROR_BAD_PARAMETERS;
      break;
    }
    ret = gp_widget_set_value(child, &t);
    if (ret!=GP_OK)
      gp_context_error(photo->context,"Failed to set values %s of toggle widget %s.", value, param);
    break;
  }
  case GP_WIDGET_DATE: { /* int      */
    int t = -1;
#ifdef HAVE_STRPTIME
    struct tm xtm;

    if (strptime (value, "%c", &xtm) || strptime (value, "%Ec", &xtm))
    t = mktime (&xtm);
#endif
    if (t==-1) {
      if (!sscanf(value, "%d", &t)) {
        gp_context_error(photo->context,"The passed value %s is neither a valid time nor an integer.", value);
        ret = GP_ERROR_BAD_PARAMETERS;
        break;
      }
    }
    ret = gp_widget_set_value(child, &t);
    if (ret!=GP_OK)
      gp_context_error(photo->context,"Failed to set new time of date/time widget %s to %s.", param, value);
    break;
  }
  case GP_WIDGET_MENU:
  case GP_WIDGET_RADIO: { /* char *   */
    int cnt, i;

    cnt = gp_widget_count_choices(child);
    if (cnt<GP_OK) {
      ret = cnt;
      break;
    }
    ret = GP_ERROR_BAD_PARAMETERS;
    for(i = 0; i<cnt; i++) {
      const char *choice;

      ret = gp_widget_get_choice(child, i, &choice);
      if (ret!=GP_OK)
        continue;
      if (!strcmp(choice, value)) {
        ret = gp_widget_set_value(child, value);
        break;
      }
    }
    if (i!=cnt)
      break;

    if (sscanf(value, "%d", &i)) {
      if ((i>=0)&&(i<cnt)) {
        const char *choice;

        ret = gp_widget_get_choice(child, i, &choice);
        if (ret==GP_OK)
          ret = gp_widget_set_value(child, choice);
        break;
      }
    }
    gp_context_error(photo->context,"Choice %s not found within list of choices.", value);
    break;
  }

    /* ignore: */
  case GP_WIDGET_WINDOW:
  case GP_WIDGET_SECTION:
  case GP_WIDGET_BUTTON:
    gp_context_error(photo->context,"The %s widget is not configurable.", param);
    ret = GP_ERROR_BAD_PARAMETERS;
    break;
  }
  if (ret==GP_OK) {
    ret = gp_camera_set_config(photo->cam, rootconfig, photo->context);
    if (ret!=GP_OK)
      gp_context_error(photo->context,"Failed to set new configuration value %s for configuration entry %s.", value, param);
  }
  gp_widget_free(rootconfig);
  return (ret==GP_OK);
}

int photo_get_config(photo_p photo, const char *param, char **value)
{
  CameraWidget *rootconfig, *child;
  int ret;
  const char *label;
  CameraWidgetType type;

  /* find widget */
  ret = find_widget_by_name(photo->cam, photo->context, param, &child, &rootconfig);
  if (ret != GP_OK)
    return 0;

  /* get widget type */
  ret = gp_widget_get_type (child, &type);
  if (ret != GP_OK) {
    gp_widget_free (rootconfig);
    return 0;
  }

  /* get widget label */
  ret = gp_widget_get_label (child, &label);
  if (ret != GP_OK) {
    gp_widget_free (rootconfig);
    return 0;
  }

//  printf ("Label: %s\n", label); /* "Label:" is not i18ned, the "label" variable is */
  switch (type) {
  case GP_WIDGET_TEXT: {    /* char *   */
    char *txt;

    ret = gp_widget_get_value (child, &txt);
    if (ret != GP_OK) {
      gp_context_error (photo->context,"Failed to retrieve value of text widget %s.", param);
    }
    *value = txt;
    break;
  }
  case GP_WIDGET_RANGE: { /* float    */
    float f, t,b,s;

    ret = gp_widget_get_range (child, &b, &t, &s);
    if (ret != GP_OK){
      gp_context_error (photo->context,"Failed to retrieve values of range widget %s.", param);
    }
    ret = gp_widget_get_value (child, &f);
    if (ret != GP_OK) {
      gp_context_error (photo->context,"Failed to value of range widget %s.", param);
    }
    sprintf(*value,"%f",f);
    break;
  }
  case GP_WIDGET_TOGGLE: {  /* int    */
    int t;

    ret = gp_widget_get_value (child, &t);
    if (ret != GP_OK) {
      gp_context_error (photo->context,"Failed to retrieve values of toggle widget %s.", param);
    }
    sprintf(*value,"%d",t);
    break;
  }
  case GP_WIDGET_DATE:  {   /* int      */
    int ret, t;
    time_t  xtime;
    struct tm *xtm;
    char  timebuf[200];

    ret = gp_widget_get_value (child, &t);
    if (ret != GP_OK) {
      gp_context_error (photo->context,"Failed to retrieve values of date/time widget %s.", param);
      break;
    }
    xtime = t;
    xtm = localtime (&xtime);
    ret = strftime (timebuf, sizeof(timebuf), "%c", xtm);
    sprintf(*value,"%s",timebuf);
    break;
  }
  case GP_WIDGET_MENU:
  case GP_WIDGET_RADIO: { /* char *   */
    char *current;
    ret = gp_widget_get_value (child, &current);
    if (ret != GP_OK) {
      gp_context_error (photo->context,"Failed to retrieve values of radio widget %s.", param);
    }
    sprintf(*value,"%s",current);
    break;
  }

  /* ignore: */
  case GP_WIDGET_WINDOW:
  case GP_WIDGET_SECTION:
  case GP_WIDGET_BUTTON:
    break;
  }
  gp_widget_free (rootconfig);
  return (ret==GP_OK);
}


