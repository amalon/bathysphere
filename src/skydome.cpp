/*
 * skydome.cpp
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
 * The sky and sun.
 *
 */

#include "skydome.h"
#include "object.h"
#include "environment.h"
#include "observer.h"
#include "glmaths.h"
#include "texload.h"
#include "constants.h"

#include <cstdlib>
#include <cstdio>
#include <GL/glu.h>

/// Constructor
Skydome::Skydome()
  : _skyColour(66.0f/255, 135.0f/255, 236.0f/255),
    _horizonColour(139.0f/255, 166.0f/255, 255.0f/255),
    _sunTexture(0)
{
  // load the sun texture
  {
    const char * filename = DATA_DIRECTORY "sun.rgb";
    int width, height;
    GLubyte * data = read_rgb_texture(filename, &width, &height);
    if (data) {
        // bind the texture and send the data to opengl.
      glGenTextures(1, &_sunTexture);
      glBindTexture(GL_TEXTURE_2D, _sunTexture);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, width, height,
                        GL_RGB, GL_UNSIGNED_BYTE, data);
      free(data);
    } else {
        // problems loading
      printf("Error: couldn't load sun texture %s\n", filename);
    }
  }
}

/// Draw the sky dome
void Skydome::Render(const Observer & observer)
{
#define RINGS    12
#define SECTIONS (RINGS << 1)
  float radius = 100.0f;
  
  // Draw a large hemispherical sky, going slightly under the horizon
  glPushMatrix();
  {
    glTranslate(observer.GetPosition());
    glScalef(radius, radius, radius);
    
    // Don't write to the depth buffer or depth test
    glDisable(GL_DEPTH_TEST);
    glDepthMask(0);
    // We don't want lighting for the sky since its an emitter
    glDisable(GL_LIGHTING);
    
#if 0
    float sincosEl[2][2];
    maths::Vector<3, float> colour[2];
    int alternator = 0;
    sincosEl[alternator][0] = 0.0f;
    sincosEl[alternator][1] = 1.0f;
    colour[alternator] = _horizonColour;
    alternator = 1 - alternator;
    for (int i = 1; i <= RINGS; ++i) {
      float factor = (float)i / RINGS;
      float elevation = 0.5f * PI * (-1.0f + 2.0f * factor);
      sincosEl[alternator][0] = sin(elevation);
      sincosEl[alternator][1] = cos(elevation);
      colour[alternator] = _horizonColour * (1.0f - factor) + _skyColour * factor;
      glBegin(GL_TRIANGLE_STRIP);
      {
        for (int j = 0; j <= SECTIONS; ++j) {
          float bearing = 2.0f * PI * j / SECTIONS;
          float sinBe = sin(bearing);
          float cosBe = cos(bearing);
          
          gl::Color3(colour[alternator]);
          glVertex3f(sincosEl[alternator][1] * sinBe,
                     sincosEl[alternator][1] * cosBe,
                     sincosEl[alternator][0]);
          gl::Color3(colour[1-alternator]);
          glVertex3f(sincosEl[1-alternator][1] * sinBe,
                     sincosEl[1-alternator][1] * cosBe,
                     sincosEl[1-alternator][0]);
        }
      }
      glEnd();
      
      alternator = 1 - alternator;
    }
#endif
    
    RenderSunGlare();
    
    // Restore normal depth buffer mode
    glEnable(GL_DEPTH_TEST);
    glDepthMask(1);
    glEnable(GL_LIGHTING);
  }
  glPopMatrix();
}

/// Render the sun glare.
void Skydome::RenderSunGlare()
{
  maths::Vector<3, float> sunPos(Object::GetDefaultEnvironment()->GetSunDirection());
  float scale = 0.5f;
  maths::Vector<3, float> center = sunPos * scale;
  
  glBindTexture(GL_TEXTURE_2D, GetSunTexture());
  glEnable(GL_TEXTURE_2D);
  
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE);
  GLboolean fogEnabled = glIsEnabled(GL_FOG);
  if (fogEnabled) {
    glDisable(GL_FOG);
  }
  
  //scale *= 0.5f;
  float intensity = 1.0f;
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
  if (fogEnabled) {
    glEnable(GL_FOG);
  }
  
  glDisable(GL_TEXTURE_2D);
}
