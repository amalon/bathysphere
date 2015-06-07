/*
 * kelp.cpp
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
 * Giant kelp object.
 *
 */

#include "kelp.h"
#include "glmaths.h"

#include <cstdlib>

/// Cached leaf widths.
float Kelp::KelpLeaf::s_leafWidths[KELP_LEAF_LENGTH_SECTIONS];

/// Default constructor.
Kelp::Kelp(float length, float baseRadius, const maths::Vector<3, float> & position)
  : Strand(length, 1+(int)(length/4.0f),
           1.0f, 950.0f,
           0.5f, position, maths::Vector<3, float>(0.0f, 0.0f, 1.0f)),
    _baseRadius(baseRadius),
    _baseExtension(0.0f)
{
  unsigned int i;
  /// set the volume of each bit
  float prevHeight = 0.0f;
  float prevRadius = GetRadiusAtHeight(0.0f);
  for (i = 0; i < _numControlPoints; ++i) {
    float height = prevHeight + _controlPoints[i].GetRadius();
    float radius = GetRadiusAtHeight(height);
    // 1/3 PI * (prevradius*prevradius*(_length-prevheight) - radius*radius*(_length-height))
    float volume = (PI/3)*(prevRadius*prevRadius*(_length-prevHeight) - radius*radius*(_length-height));
    _controlPoints[i].SetVolume(volume);
    prevHeight = height;
    prevRadius = radius;
  }
  /// Add leaves
  unsigned int numLeaves = (int)(length*20.0f);
  for (i = 0; i < numLeaves; ++i) {
    float height = length * ((float)rand() / RAND_MAX);
    float leaflen = 1.0f + 2.0f * ((float)rand() / RAND_MAX);
    
    KelpLeaf * leaf = new KelpLeaf(leaflen, InterpolateStrand(height),
                                   maths::Vector<3, float> (-1.0f + 2.0f*((float)rand() / RAND_MAX),
                                       -1.0f + 2.0f*((float)rand() / RAND_MAX),
                                           -1.0f + 2.0f*((float)rand() / RAND_MAX)));
    AddSubStrand(leaf, height);
  }
  /// Occasionally add subkelp
  if (rand() % 2 != 0) {
    float height = 0.2f + 0.3f * length * ((float)rand() / RAND_MAX);
    float newLength = 0.4f + 0.4f * length * ((float)rand() / RAND_MAX);
    Kelp * kelp = new Kelp(newLength, GetRadiusAtHeight(height)*0.8f, InterpolateStrand(height));
    AddSubStrand(kelp, height);
  }
  
  // Have some random movement
  _randomMovement = 2.0f;
}

/// Get the radius at a particular height.
float Kelp::GetRadiusAtHeight(float height)
{
  return _baseRadius * (1.0f - height / _length);
}
#include <cassert>

/// Draw the strand using the derived classes functions.
void Kelp::VRenderStrand(const Observer & observer)
{
  {
    static float diffuse[4] = {0.5f, 0.7f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
    static float zero[4]  = {0.0f, 0.0f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, zero);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
  }
  
  if (true) {
#define KELP_RING_SEGMENTS 16
    // Give it some thickness
    maths::Vector<3, float> prevPos = GetPosition();
    float prevRadius = GetRadiusAtHeight(0.0f);
    if (_baseExtension) {
      // Draw a little bit underneith to cover for if the terrain adjusts to lower resolution
      glBegin(GL_TRIANGLE_STRIP);
      {
        for (int j = 0; j <= KELP_RING_SEGMENTS; ++j) {
          float ang = 2.0f*PI*j/KELP_RING_SEGMENTS;
          float sinang = sin(ang);
          float cosang = cos(ang);
          glNormal3f(sinang, cosang, 0.0f);
          glVertex3f(prevPos[0] + prevRadius*sinang, prevPos[1] + prevRadius*cosang, prevPos[2] - _baseExtension);
          glVertex3f(prevPos[0] + prevRadius*sinang, prevPos[1] + prevRadius*cosang, prevPos[2]);
        }
      }
    }
    
    int segments = (int)(3.0f*_length);
    for (int i = 1; i <= segments; ++i) {
      float height = _length * i/segments;
      maths::Vector<3, float> pos = InterpolateStrand(height);
      float radius = GetRadiusAtHeight(height);
      glBegin(GL_TRIANGLE_STRIP);
      {
        for (int j = 0; j <= KELP_RING_SEGMENTS; ++j) {
          float ang = 2.0f*PI*j/KELP_RING_SEGMENTS;
          float sinang = sin(ang);
          float cosang = cos(ang);
          glNormal3f(sinang, cosang, 0.0f);
          glVertex3f(prevPos[0] + prevRadius*sinang, prevPos[1] + prevRadius*cosang, prevPos[2]);
          glVertex3f(pos[0] + radius*sinang, pos[1] + radius*cosang, pos[2]);
        }
      }
      glEnd();
      prevPos = pos;
      prevRadius = radius;
    }
  } else if (true) {
    // Do a basic line up the kelp
    glBegin(GL_LINE_STRIP);
    glVertex3(GetPosition());
    for (int i = 0; i < 100; ++i) {
      float height = _length * i/100;
      glVertex3(InterpolateStrand(height));
    }
    glVertex3(_controlPoints[_numControlPoints-1].GetPosition());
    glEnd();
  }
  // Show control points
  if (false) {
    glDisable(GL_DEPTH_TEST);
    glPointSize(5);
    glBegin(GL_POINTS);
    {
      glVertex3(GetPosition());
      for (unsigned int i = 0; i < _numControlPoints; ++i) {
        glVertex3(_controlPoints[i].GetPosition());
      }
    }
    glEnd();
    glPointSize(1);
    glEnable(GL_DEPTH_TEST);
  }
}

/// Default constructor.
Kelp::KelpLeaf::KelpLeaf(float length, const maths::Vector<3, float> & position,
                         const maths::Vector<3, float> & direction)
  : Strand(length, 2 + (rand() % 2) * (rand() % 2),
           1.0f, 1020.0f,
           0.5f, position, direction),
    _diffuse(0.6f * rand()/RAND_MAX,
             0.6 + 0.4f * rand()/RAND_MAX,
             0.2f * rand()/RAND_MAX,
             1.0f )
{
  /// Calculate the leaf widths
  static bool leafWidthsCalculated = false;
  if (!leafWidthsCalculated) {
#define LEAF_BASE_LENGTH     4
#define LEAF_END_LENGTH      4
    for (int i = 0; i < KELP_LEAF_LENGTH_SECTIONS; ++i) {
      float width;
      if (i < LEAF_BASE_LENGTH) {
        width = sin(0.5f*PI*i/LEAF_BASE_LENGTH);;
      } else if (i >= (KELP_LEAF_LENGTH_SECTIONS - LEAF_END_LENGTH)) {
        width = 0.5f*(1.0f-cos(PI*(KELP_LEAF_LENGTH_SECTIONS - i - 1) / LEAF_END_LENGTH));
      } else {
        width = 1.0f;
      }
      s_leafWidths[i] = width;
    }
    leafWidthsCalculated = true;
  }
  
  // Have a fair bit of random movement.
  _randomMovement = 5.0f;
  
  // if the leaf is partly transparent, then enable alpha blending
  if (_diffuse[3] < 1.0f) {
    _alpha = true;
  }
  
}

/// Draw the kelp leaf
void Kelp::KelpLeaf::VRenderStrand(const Observer & observer)
{
  {
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, _diffuse);
    static float zero[4]  = {0.0f, 0.0f, 0.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, zero);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
  }
  
  // if the leaf is partly transparent, then enable alpha blending
  if (_diffuse[3] < 1.0f) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }
  
  if (true) {
    // flag simple shape to test use of trig functions in shape
    maths::Vector<3, float> direction = _controlPoints[0].GetPosition();
    direction -= GetPosition();
    direction.normalize();
    maths::Vector<3,float> up(0.0f, 0.0f, 1.0f);
    maths::Vector<3,float> side;
    maths::cross(side, direction,up);
    side.normalize();
    maths::cross(up, side, direction);
    glDisable(GL_CULL_FACE);
    glBegin(GL_TRIANGLE_STRIP);
    {
      for (int i = 0; i < KELP_LEAF_LENGTH_SECTIONS; ++i) {
        float height = _length * i/KELP_LEAF_LENGTH_SECTIONS;
        maths::Vector<3,float> pos = (InterpolateStrand(height));
        float width = 0.1f * s_leafWidths[i];
        glNormal(up);
        glVertex3(pos+side*width);
        glVertex3(pos-side*width);
      }
    }
    glEnd();
    glEnable(GL_CULL_FACE);
    
  } else if (false) {
    glVertex3(GetPosition());
    for (int i = 0; i < 10; ++i) {
      float height = _length * i/10;
      glVertex3(InterpolateStrand(height));
    }
    glEnd();
  } else {
    // Do a basic line up the kelp
    glBegin(GL_LINE_STRIP);
    glVertex3(GetPosition());
    for (int i = 0; i < 10; ++i) {
      float height = _length * i/10;
      glVertex3(InterpolateStrand(height));
    }
    glVertex3(_controlPoints[_numControlPoints-1].GetPosition());
    glEnd();
  }
  
  if (_diffuse[3] < 1.0f) {
    glDisable(GL_BLEND);
  }
  
  // Show control points
#if 0
  glDisable(GL_DEPTH_TEST);
  glPointSize(5);
  glBegin(GL_POINTS);
  {
    glVertex3(GetPosition());
    for (unsigned int i = 0; i < _numControlPoints; ++i) {
      glVertex3(_controlPoints[i].GetPosition());
    }
  }
  glEnd();
  glPointSize(1);
  glEnable(GL_DEPTH_TEST);
#endif
}
