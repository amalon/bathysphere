/*
 * cord.cpp
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
 * A tether object.
 *
 */

#include "cord.h"
#include "environment.h"
#include "texload.h"
#include "glmaths.h"

#include <cstdlib>
#include <cstdio>
#include <GL/glu.h>

/// Default constructor.
Cord::Cord(float length, const maths::Vector<3, float> & position) :
    SimpleStrand(length, 30,
                 PI*0.02f*0.02f, 7850.0f,
                 0.5f, position, maths::Vector<3, float>(0.0f, 0.0f, -1.0f)),
    _cordRadius(0.05f),
    _textureHeight(0.3f)
{
  // Load the coral texture
  const char * filename = DATA_DIRECTORY "cable.rgb";
  glGenTextures(1, &_texture);
  GLubyte *data;
  // load the file
  int width, height;
  data = read_rgb_texture(filename, &width, &height);
  if (data) {
    // bind the texture and send the data to opengl.
    glBindTexture(GL_TEXTURE_2D, _texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, width, height,
                      GL_RGB, GL_UNSIGNED_BYTE, data);
    free(data);
  } else {
    // problems loading
    printf("Error: couldn't load cable texture %s\n", filename);
    exit(1);
  }
}

/// Destructor
Cord::~Cord()
{
}

/// Set up the material and texture.
void Cord::SetupMaterial()
{
  {
    static float diffuse[4]  = {0.7f, 0.7f, 0.7, 1.0f};
    static float specular[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 120.0f);
  }
    // Bind the texture
  glBindTexture(GL_TEXTURE_2D, _texture);
}

/// Draw the strand using the derived classes functions.
void Cord::VRenderStrand(const Observer & observer)
{
  if (true) {
    SetupMaterial();
    glEnable(GL_TEXTURE_2D);
    
    maths::Vector<3, float> prevPos = GetPosition();
    float radius = _cordRadius;
    float oldHeight = 0.0f;
#define RENDER_SECTIONS 100
    for (int i = 1; i <= RENDER_SECTIONS; ++i) {
      float height = _length * i/RENDER_SECTIONS;
      maths::Vector<3, float> pos;
      if (i == RENDER_SECTIONS) {
        pos = _controlPoints[_numControlPoints-1].GetPosition();
      } else {
        pos = InterpolateStrand(height);
      }
      glBegin(GL_TRIANGLE_STRIP);
      for (int j = 0; j <= 16; ++j) {
        float ang = 2.0f*PI*j/16;
        float sinang = sin(ang);
        float cosang = cos(ang);
        glNormal3f(sinang, cosang, 0.0f);
        glTexCoord2f((float)j/16, (pos[2]-_controlPoints[_numControlPoints-1].GetPosition()[2])/_textureHeight);
        glVertex3f(pos[0] + radius*sinang, pos[1] + radius*cosang, pos[2]);
        glTexCoord2f((float)j/16, (prevPos[2]-_controlPoints[_numControlPoints-1].GetPosition()[2])/_textureHeight);
        glVertex3f(prevPos[0] + radius*sinang, prevPos[1] + radius*cosang, prevPos[2]);
      }
      glEnd();
      prevPos = pos;
      oldHeight = height;
    }
    
    glDisable(GL_TEXTURE_2D);
  } else if (true) {
    // Do a basic line up the cord (temporary)
    glDisable(GL_LIGHTING);
    glLineWidth(4);
    glBegin(GL_LINE_STRIP);
    glVertex3(GetPosition());
    unsigned int controlPoint = 0;
    //float lowHeight = 0.0f;
    float topHeight = _controlPoints[0].GetRadius();
    for (int i = 0; i < 100; ++i) {
      float height = _length * i/100;
      if (height > topHeight && controlPoint < _numControlPoints-1) {
        ++controlPoint;
        //lowHeight = topHeight;
        topHeight += _controlPoints[controlPoint].GetRadius();
      }
      float tension = _controlPoints[controlPoint].tension.sqr()/3000;
      glColor3f(tension,tension,tension);
      glVertex3(InterpolateStrand(height));
    }
    glVertex3(_controlPoints[_numControlPoints-1].GetPosition());
    glEnd();
    glEnable(GL_LIGHTING);
    glLineWidth(1);
  }
}

