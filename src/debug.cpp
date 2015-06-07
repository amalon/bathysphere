/*
 * debug.cpp
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
 * Debug object which times out.
 *
 */

#include "debug.h"
#include "primitives.h"

#include <GL/gl.h>

/// Whether to draw debug objects.
bool Debug::DebugObject::_enableDebugRender = false;

/// Main constructor.
Debug::DebugObject::DebugObject(float timeout)
  : _timeout(timeout)
{
}

/// Advance the timeout
void Debug::DebugObject::VAdvance(float dt)
{
  _timeout -= dt;
  if (_timeout < 0.0f) {
    Delete();
  }
}


/// Main constructor.
Debug::Axes::Axes(float size, float timeout)
  : DebugObject(timeout)
{
  _radius = size;
}

/// Draw the axes
void Debug::Axes::VDebugRender(const Observer & observer)
{
  // Blend if about to disapear
  float alpha = _timeout*10.0f;
  if (alpha >= 1.0f) {
    alpha = 1.0f;
  } else {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }
  
  // The colours of the axes
  GLfloat colours[3][4] = {
    { 1.0f, 0.0f, 0.0f, alpha },
    { 0.0f, 1.0f, 0.0f, alpha },
    { 0.0f, 0.0f, 1.0f, alpha }
  };
  
  // Remove any previous shininess
  {
    static float zero[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, zero);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
  }
  
  // Draw the arrows
  glPushMatrix();
  {
    glScalef(GetRadius(), GetRadius(), GetRadius());
    
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, colours[0]);
    Drawing::Arrow(0.02f, 0.05f, 0.2f, 1.0f, -1.0f);
    
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, colours[1]);
    glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
    Drawing::Arrow(0.02f, 0.05f, 0.2f, 1.0f, -1.0f);
    
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, colours[2]);
    glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
    Drawing::Arrow(0.02f, 0.05f, 0.2f, 1.0f, -1.0f);
  }
  glPopMatrix();
  
  if (alpha < 1.0f) {
    glDisable(GL_ALPHA);
  }
}
