/*
 * collisionresponse.cpp
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
 * Collision impact reaction force arrow object.
 *
 */

#include "collisionresponse.h"
#include "primitives.h"

#define LIFETIME 1.0f

/// Default constructor.
Collision::Response::Response(const maths::Vector<3, float> & pos, const maths::Vector<3, float> & normal, const maths::Vector<3, float> & force)
  : DebugObject(LIFETIME)
{
  _radius = 0.5f;
  SetPosition(pos);
  // Calculate the orientation
  _orientation.SetQuaternion(rotationArc(maths::Vector<3, float>(0.0f, 0.0f, 1.0f), normal));
  _force = force * _orientation.GetMatrix();
}

/// Draw a simple plane and normal vector.
void Collision::Response::VDebugRender(const Observer & observer)
{
  _orientation.ApplyGlTranformation();
  
  // Blend if about to disapear
  float alpha =_timeout/LIFETIME;
  if (alpha >= 1.0f) {
    alpha = 1.0f;
  } else {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }
  
  {
    // Remove any previous shininess
    static float zero[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    float ambdiff[4] = { 1.0f, 1.0f, 0.0f, alpha };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, ambdiff);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, zero);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
  }
  
  // Draw the arrows
  glPushMatrix();
  {
    float radius = GetRadius();
    glScalef(radius, radius, radius);
    
    glLineWidth(1.5f);
    glNormal3f(0.0f, 0.0f, 1.0f);
    if (false) {
      // rectangle on plane
      glBegin(GL_LINE_LOOP);
      {
        glVertex3f( 1.0f, 1.0f, 0.0f);
        glVertex3f(-1.0f, 1.0f, 0.0f);
        glVertex3f(-1.0f,-1.0f, 0.0f);
        glVertex3f( 1.0f,-1.0f, 0.0f);
      }
      glEnd();
    }
    // cross through point of contact on plane
    glBegin(GL_LINES);
    {
      glVertex3f( 1.0f, 1.0f, 0.0f);
      glVertex3f(-1.0f,-1.0f, 0.0f);
      glVertex3f(-1.0f, 1.0f, 0.0f);
      glVertex3f( 1.0f,-1.0f, 0.0f);
    }
    glEnd();
    glLineWidth(1.0f);
    
    Drawing::Arrow(0.01f, 0.2f, 0.2f, _force[2]*0.0001f, 0.0f);
    
    glPushMatrix();
    {
      static float ambdiff[4] = { 1.0f, 0.0f, 1.0f, alpha };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, ambdiff);
      float force = _force[0]*0.0001f;
#if 0
      glBegin(GL_LINES);
      {
        glVertex3f(0,0,0);
        glVertex3f(force,0,0);
      }
      glEnd();
#endif
      if (force < 0.0f) {
        force = -force;
        glRotatef(-90.0f, 0.0f, 1.0f, 0.0f);
      } else {
        glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
      }
      Drawing::Arrow(0.01f, 0.2f, 0.2f, force, 0.0f);
    }
    glPopMatrix();
    
    glPushMatrix();
    {
      static float ambdiff[4] = { 0.0f, 1.0f, 1.0f, alpha };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, ambdiff);
      float force = _force[1]*0.0001f;
#if 0
      glBegin(GL_LINES);
      {
        glVertex3f(0,0,0);
        glVertex3f(0,force,0);
      }
      glEnd();
#endif
      if (force < 0.0f) {
        force = -force;
        glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
      } else {
        glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
      }
      Drawing::Arrow(0.01f, 0.2f, 0.2f, force, 0.0f);
    }
    glPopMatrix();
    
  }
  glPopMatrix();
  
  if (alpha < 1.0f) {
    glDisable(GL_ALPHA);
  }
}
