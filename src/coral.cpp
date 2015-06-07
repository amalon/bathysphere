/*
 * coral.cpp
 *
 * Copyright (C) 2007-2014 James Hogan <james@albanarts.com>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details
 * (in the file called COPYING).
 *
 *
 * A coral colony object.
 *
 */

#include "coral.h"
#include "texload.h"

#include <cstdlib>
#include <cstdio>
#include <cassert>
#include <GL/glu.h>

/// The coral texture id.
GLuint Coral::s_texture = 0;
/// References to the coral texture.
unsigned int Coral::s_textureRefs = 0;

/// Constructor.
Coral::CoralModel::CoralModel()
  : _displayList(0),
    _references(0)
{
  _displayList = glGenLists(1);
}

/// Destructor.
Coral::CoralModel::~CoralModel()
{
  if (_displayList) {
    glDeleteLists(_displayList, 1);
  }
}

/// Render the display list.
void Coral::CoralModel::Render(const Observer & observer)
{
  if (_displayList) {
    glCallList(_displayList);
  }
}

/// Start compiling the display list.
bool Coral::CoralModel::BeginCompile()
{
  if (_displayList) {
    glNewList(_displayList, GL_COMPILE);
    return true;
  } else {
    return false;
  }
}

/// Finish compiling the display list.
bool Coral::CoralModel::EndCompile()
{
  if (_displayList) {
    glEndList();
    return true;
  } else {
    return false;
  }
}

/// Constructor.
Coral::Coral(CoralModel * model)
  : _model(model),
    _diffuse((float)rand() / RAND_MAX,
             (float)rand() / RAND_MAX,
             (float)rand() / RAND_MAX, 1.0f)
{
  _radius = 3.0f;
  if (_model) {
    _model->AddReference();
  } else {
    assert(0 && "Coral constructor model parameter must point to a CoralModel");
  }
  
  // Load the texture if not already loaded.
  if (!s_textureRefs) {
    // Load the coral texture
    const char * filename = DATA_DIRECTORY "searod.rgb";
    glGenTextures(1, &s_texture);
    GLubyte *data;
    // load the file
    int width, height;
    data = read_rgb_texture(filename, &width, &height);
    if (data) {
      // bind the texture and send the data to opengl.
      glBindTexture(GL_TEXTURE_2D, s_texture);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, width, height,
                        GL_RGB, GL_UNSIGNED_BYTE, data);
      free(data);
    } else {
      // problems loading
      printf("Error: couldn't load reef texture %s\n", filename);
      exit(1);
    }
  }
  // Increment the texture reference counter.
  ++s_textureRefs;
}

/// Destructor.
Coral::~Coral()
{
  if (_model) {
    _model->RemoveReference();
  }
  
  // Remove the texture
  if (!--s_textureRefs) {
    glDeleteTextures(1, &s_texture);
  }
}

/// Render the coral model.
void Coral::VRender(const Observer & observer)
{
  if (_model) {
    // Coral
    {
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, _diffuse);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, _diffuse);
      glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
    }
    if (s_texture) {
      // Enable texturing
      glEnable(GL_TEXTURE_2D);
      glBindTexture(GL_TEXTURE_2D, s_texture);
    }
    _model->Render(observer);
    if (s_texture) {
      // Disable texturing
      glDisable(GL_TEXTURE_2D);
    }
  }
}
