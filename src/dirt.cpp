/*
 * dirt.cpp
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
 * Dirt floating around in the water.
 *
 */

#include "dirt.h"
#include "observer.h"
#include "glmaths.h"

Dirt::Dirt()
{
  // Model as a sphere of air
  _radius = 0.01f;
  _volume = (4.0f/3.0f*PI)*_radius*_radius*_radius;
  _density = 1000.025f; // aprox 1.2kg/m3
}

// Render the object
void Dirt::VRender(const Observer & observer)
{
  {
    static float diffuse[4]  = {0.0f, 0.0f, 0.0f, 1.0f};
    static float specular[4] = {0.0f, 0.5f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 80.0f);
  }
  
  glNormal(observer.GetDirection());
  //glDisable(GL_LIGHTING);
  glPointSize(1.5f);
  glBegin(GL_POINTS);
  {
    //glColor3f(0.0f, 0.5f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
  }
  glEnd();
  glPointSize(1.0f);
  //glEnable(GL_LIGHTING);
}
