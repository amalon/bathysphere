/*
 * seabed.cpp
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
 * The bottom of the seabed.
 *
 */

#include "seabed.h"
#include "glmaths.h"
#include "constants.h"

Seabed::Seabed(float size)
  : _halfSize(size/2)
{
  _radius = size*Sqrt3;
}
    
// Render the object (translating as appropriate then calling virtual render function)
void Seabed::VRender(const Observer & observer)
{
  {
    float diffuse[4] = {1.0f, 1.0f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
    static float zero[4]  = {0.0f, 0.0f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, zero);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
  }
  
  float inner = _halfSize;
  float outer = inner*2.0f;
  
  glNormal3f(0.0f, 0.0f, 1.0f);
  
  glBegin(GL_TRIANGLE_STRIP);
  {
    glVertex3f(-inner, -inner, 0.0f);
    glVertex3f( inner, -inner, 0.0f);
    glVertex3f(-inner,  inner, 0.0f);
    glVertex3f( inner,  inner, 0.0f);
  }
  glEnd();
  
  // Using homogenous coordinates to the seabed go infinitely doesn't appear
  // to work. probably something to do with the construction of the perspective
  // matrix
  for (int i = 0; i < 3; ++i) {
    glBegin(GL_TRIANGLE_STRIP);
    {
      glVertex3f(-inner, -inner, 0.0f);
      glVertex3f(-outer, -outer, 0.0f);
      
      glVertex3f( inner, -inner, 0.0f);
      glVertex3f( outer, -outer, 0.0f);
      
      glVertex3f( inner,  inner, 0.0f);
      glVertex3f( outer,  outer, 0.0f);
      
      glVertex3f(-inner,  inner, 0.0f);
      glVertex3f(-outer,  outer, 0.0f);
      
      glVertex3f(-inner, -inner, 0.0f);
      glVertex3f(-outer, -outer, 0.0f);
    }
    glEnd();
    inner = outer;
    outer *= 2.0f;
  }
}
