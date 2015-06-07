/*
 * seasurface.cpp
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
 * The waters surface.
 *
 */

#include "seasurface.h"
#include "environment.h"
#include "glmaths.h"
#include "constants.h"

#include <cstdlib>

SeaSurface::SeaSurface(float size)
  : _halfSize(size/2),
    _sunReflectLight(GL_LIGHT7),
    _drawSunRays(true)
{
  _radius = size*Sqrt3;
  _alpha = true;
  
  int i,j;
  for (i = 0; i < LIGHT_RAY_QUANTITY; ++i) {
    for (j = 0; j < LIGHT_RAY_QUANTITY; ++j) {
      _rayDisplacements[i][j][0] = -5.0f + 10.0f*rand()/RAND_MAX;
      _rayDisplacements[i][j][1] = -5.0f + 10.0f*rand()/RAND_MAX;
      _rayDisplacements[i][j][2] = -50.0f + 40.0f*rand()/RAND_MAX;
    }
  }
  
  GLfloat sun_amb_and_diff[] = {0.0f, 0.0f, 0.0f, 1.0f};
  GLfloat specular[]         = {1.0f, 1.0f, 1.0f, 1.0f};

  glLightfv(_sunReflectLight, GL_AMBIENT, sun_amb_and_diff);
  glLightfv(_sunReflectLight, GL_DIFFUSE, sun_amb_and_diff);
  glLightfv(_sunReflectLight, GL_SPECULAR, specular);
}
    
// Render the object (translating as appropriate then calling virtual render function)
void SeaSurface::VRender(const Observer & observer)
{
  RenderSurface(observer);
  if (_drawSunRays) {
    RenderSunRays(observer);
  }
  RenderSunGlare(observer);
}

/// Get the altitude of the water surface in scene coordinate space.
float SeaSurface::GetAltitude(const maths::Vector<2, float> & position) const
{
  return GetPosition()[2]
      + Object::s_defaultEnvironment->GetIntegratedCurrent(GetPosition() + maths::Vector<3, float>(position))[2];
}

/// Render the surface of the water.
void SeaSurface::RenderSurface(const Observer & observer)
{
  bool above = Object::s_defaultEnvironment->GetAboveWater();
  {
    static float diffuseBelow[4] = {0.5f, 1.0f, 1.0f, 1.0f};
    static float diffuseAbove[4] = {0.2f, 0.4f, 0.6f, 1.0f};
    static float specular[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    if (above) {
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuseAbove);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, diffuseAbove);
    } else {
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuseBelow);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, diffuseBelow);
    }
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 128.0f);
  }
  
  // do a trick and duplicate the sun, reflected by the plane of the water.
  // if the sea surface is shiny, it should reflect the sun where it would
  // ordinarily shine through.
  
  // get the light position
  maths::Vector<4,float> sunReflectPos = Object::s_defaultEnvironment->GetSunDirection();
  sunReflectPos[2] = - sunReflectPos[2];
  glLightfv(_sunReflectLight, GL_POSITION, sunReflectPos);
  glEnable(_sunReflectLight);
  
  float inner = _halfSize;
  float outer = inner*2.0f;
  
  Object::s_defaultEnvironment->BeginCaustics();
  
  glDisable(GL_CULL_FACE);
  // Only transparent from below
  glEnable(GL_BLEND);
  if (!above) {
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  } else {
    glBlendFunc(GL_SRC_ALPHA, GL_ZERO);
  }
  
  // Don't draw in as much detail if far away
  maths::Vector<3,float> camPos = observer.GetPosition() - GetPosition();
#define SQUASH_START -20.0f
#define SQUASH_END -40.0f
  if (camPos[2] > SQUASH_END) {
    bool squash = false;//camPos[2] < SQUASH_START;
    if (squash) {
      float scaleFactor = (camPos[2] - SQUASH_END) / (SQUASH_START - SQUASH_END);
      glPushMatrix();
      glScalef(1.0f, 1.0f, scaleFactor);
    }
    
    int i,j;
#define SEA_GRID_LENGTH 50
    float heightBuffer[SEA_GRID_LENGTH];
    for (j = 0; j < SEA_GRID_LENGTH; ++j) {
      heightBuffer[j] = 0.0f;
    }
    
    glNormal3f(0.0f, 0.0f, -1.0f);
    
    float x0 = _halfSize;
    for (i = 1; i < SEA_GRID_LENGTH; ++i) {
      float x = inner - (2.0f*inner/(SEA_GRID_LENGTH-1)) * i;
      glBegin(GL_TRIANGLE_STRIP);
      {
        for (j = 0; j < SEA_GRID_LENGTH; ++j) {
          float y = -inner + (2.0f*inner/(SEA_GRID_LENGTH-1)) * j;
          glVertex3f(x0, y, heightBuffer[j]);
          maths::Vector<3, float> xyz = GetPosition();
          xyz[0] += x;
          xyz[1] += y;
          if (!j || j == SEA_GRID_LENGTH-1 || i == SEA_GRID_LENGTH-1) {
            heightBuffer[j] = 0.0f;
          } else {
            heightBuffer[j] = Object::s_defaultEnvironment->GetIntegratedCurrent(xyz)[2];
          }
          glVertex3f(x, y, heightBuffer[j]);
        }
      }
      glEnd();
      x0 = x;
    }
    if (squash) {
      glPopMatrix();
    }
    
  } else {
    // Low detail
    glBegin(GL_TRIANGLE_STRIP);
    {
      glVertex3f(-inner, -inner, 0.0f);
      glVertex3f(-inner,  inner, 0.0f);
      glVertex3f( inner, -inner, 0.0f);
      glVertex3f( inner,  inner, 0.0f);
    }
    glEnd();
  }
  
  // Using homogenous coordinates to the seabed go infinitely doesn't appear
  // to work. probably something to do with the construction of the perspective
  // matrix
  for (int i = 0; i < 3; ++i) {
    glBegin(GL_TRIANGLE_STRIP);
    {
      glVertex3f(-inner, -inner, 0.0f);
      glVertex3f(-outer, -outer, 0.0f);
      
      glVertex3f(-inner,  inner, 0.0f);
      glVertex3f(-outer,  outer, 0.0f);
      
      glVertex3f( inner,  inner, 0.0f);
      glVertex3f( outer,  outer, 0.0f);
      
      glVertex3f( inner, -inner, 0.0f);
      glVertex3f( outer, -outer, 0.0f);
      
      glVertex3f(-inner, -inner, 0.0f);
      glVertex3f(-outer, -outer, 0.0f);
    }
    glEnd();
    inner = outer;
    outer *= 2.0f;
  }
  
  glDisable(GL_BLEND);
  glEnable(GL_CULL_FACE);
  
  Object::s_defaultEnvironment->EndCaustics();
  
  {
    static float zero[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, zero);
  }
  
  glDisable(_sunReflectLight);
}

/// Render light rays shining through the surface of the water.
void SeaSurface::RenderSunRays(const Observer & observer)
{
  bool above = Object::s_defaultEnvironment->GetAboveWater();
  if (above) {
    return;
  }
  float inner = _halfSize*2.0f;
  
  // Draw light rays
  maths::Vector<3,float> sunPos(Object::s_defaultEnvironment->GetSunDirection());
  glDisable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE);
  glDepthMask(0);
  //glLineWidth(3.0f);
  int inc = 1;
  /*float intensities[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  if (camPos[2] < -30.0) {
  intensities[1] = 1.0f - (-30.0f - camPos[2]) / 25.0f;
  if (!intensities[1]) {
  inc <<= 1;
  if (camPos[2] < -55.0f) {
  inc <<= 1;
  intensities[2] = 1.0f - (-55.0f - camPos[2]) / 20.0f;
  if (camPos[2] < -75.0f) {
  inc <<= 2;
  intensities[3] = 1.0f - (-75.0f - camPos[2]) / 15.0f;
  if (camPos[2] < -90.0f) {
  inc <<= 2;
  intensities[4] = 1.0f - (-90.0f - camPos[2]) / 10.0f;
}
}
}
}*/
  glBegin(GL_TRIANGLES);
  {
    int i,j;
    for (i = 0; i < LIGHT_RAY_QUANTITY; i += inc) {
      float x = inner - (2.0f*inner/(LIGHT_RAY_QUANTITY-1)) * i;
      for (j = 0; j < LIGHT_RAY_QUANTITY; j += inc) {
        float intensity = 0.02f;
        float y = inner - (2.0f*inner/(LIGHT_RAY_QUANTITY-1)) * j;
        maths::Vector<3, float> pos1 = GetPosition();
        pos1[0] += _rayDisplacements[i][j][0];
        pos1[1] += _rayDisplacements[i][j][1];
        maths::Vector<3, float> displacement = Object::s_defaultEnvironment->GetCurrent(pos1);
        displacement[0] *= 0.5f;
        displacement[1] *= 0.5f;
        displacement[2]  = Object::s_defaultEnvironment->GetCurrent(displacement)[2];
        pos1[0] += x;
        pos1[1] += y;
        pos1 += displacement;
        //maths::Vector<3, float> pos2(pos1);
        maths::Vector<3, float> deep1(pos1);
        
        deep1 += sunPos*_rayDisplacements[i][j][2];
        
        //pos1 -= Object::s_defaultEnvironment->GetCurrent(pos1)*10.0f;
        maths::Vector<3, float> deep2(deep1);
        //pos1 -= Object::s_defaultEnvironment->GetCurrent(pos1)*2.0f;
        //pos2 += Object::s_defaultEnvironment->GetCurrent(pos2)*2.0f;
        maths::Vector<3, float> deepCurrent = Object::s_defaultEnvironment->GetCurrent(deep1);
        deep1 -= deepCurrent*4.0f;
        deep2 += deepCurrent*4.0f;
        
        glColor4f(1.0f, 1.0f, 1.0f, intensity);
        glVertex3(pos1);
        glColor4f(1.0f, 1.0f, 1.0f, 0.0f);
        glVertex3(deep1);
        glVertex3(deep2);
        //glColor4f(1.0f, 1.0f, 1.0f, intensity);
        //glVertex3(pos2);
        //glColor4f(1.0f, 1.0f, 1.0f, 0.0f);
        //glVertex3(deep2);
      }
    }
  }
  glEnd();
  //glLineWidth(1.0f);
  glDepthMask(1);
  glDisable(GL_BLEND);
  glEnable(GL_CULL_FACE);
  glEnable(GL_LIGHTING);
}

/// Render the sun glare.
void SeaSurface::RenderSunGlare(const Observer & observer)
{
  maths::Vector<3, float> sunPos(Object::s_defaultEnvironment->GetSunDirection());
  float scale = observer.GetPosition()[2] / sunPos[2];
  maths::Vector<3, float> center = observer.GetPosition() - sunPos * scale;
  
  glBindTexture(GL_TEXTURE_2D, Object::s_defaultEnvironment->skydome.GetSunTexture());
  glEnable(GL_TEXTURE_2D);
  
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE);
  GLboolean fogEnabled = glIsEnabled(GL_FOG);
  if (fogEnabled) {
    glDisable(GL_FOG);
  }
  glDisable(GL_DEPTH_TEST);
  
  glDisable(GL_LIGHTING);
  
  //scale *= 0.5f;
  float intensity = 0.2f + observer.GetPosition()[2] / 100.0f;
  glColor3f(intensity, intensity, intensity);
  
  glBegin(GL_TRIANGLE_STRIP);
  {
    glTexCoord2f(0.0f, 0.0f);
    glVertex3(center + maths::Vector<3, float>( scale, scale, 0.0f));
    glTexCoord2f(0.0f, 1.0f);
    glVertex3(center + maths::Vector<3, float>( scale,-scale, 0.0f));
    glTexCoord2f(1.0f, 0.0f);
    glVertex3(center + maths::Vector<3, float>(-scale, scale, 0.0f));
    glTexCoord2f(1.0f, 1.0f);
    glVertex3(center + maths::Vector<3, float>(-scale,-scale, 0.0f));
  }
  glEnd();
  
  scale *= 0.1f;
  intensity = 0.5f;
  glColor3f(intensity, intensity, intensity);
  
  glBegin(GL_TRIANGLE_STRIP);
  {
    glTexCoord2f(0.0f, 0.0f);
    glVertex3(center + maths::Vector<3, float>( scale, scale, 0.0f));
    glTexCoord2f(0.0f, 1.0f);
    glVertex3(center + maths::Vector<3, float>( scale,-scale, 0.0f));
    glTexCoord2f(1.0f, 0.0f);
    glVertex3(center + maths::Vector<3, float>(-scale, scale, 0.0f));
    glTexCoord2f(1.0f, 1.0f);
    glVertex3(center + maths::Vector<3, float>(-scale,-scale, 0.0f));
  }
  glEnd();
  
  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);
  if (fogEnabled) {
    glEnable(GL_FOG);
  }
  glEnable(GL_DEPTH_TEST);
  
  glDisable(GL_TEXTURE_2D);
}
