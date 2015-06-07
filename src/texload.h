/***************************************************************************
 * Computer Graphics and Visualisation Open Assessment                     *
 *  This file was not written by me. It was taken from:                    *
 *  http://www.opengl.org/resources/code/samples/mjktips/caustics/         *
 *  and is used for loading all the textures in this project               *
 ***************************************************************************/
 
/**
 * @file texload.h
 * @brief Loads rgb and bw files.
 */

/* Copyright (c) Mark J. Kilgard, 1997.  */

/* This program is freely distributable without licensing fees 
   and is provided without guarantee or warrantee expressed or 
   implied. This program is -not- in the public domain. */

#include <GL/gl.h>

/// Where the data files are.
#define DATA_DIRECTORY "data/"

extern GLubyte * read_alpha_texture(const char *name, int *width, int *height);
extern GLubyte * read_rgb_texture(const char *name, int *width, int *height);

