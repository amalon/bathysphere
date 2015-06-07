/*
 * primitives.cpp
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
 * Drawing functions for mathematical shapes.
 *
 */

#include "primitives.h"
#include "constants.h"
#include "glmaths.h"

void Drawing::Torus(float ringRadius, float tubeRadius, unsigned int ringSections, unsigned int tubeSections, float z, int tubeSectionStart, int tubeSectionEnd)
{
  unsigned int ringSection, tubeSection;
  
  if (tubeSectionEnd <= 0) {
    tubeSectionEnd = tubeSections + tubeSectionEnd;
  }
  unsigned int numTubeSections = tubeSectionEnd - tubeSectionStart + 1;
  
  // construct a lookup table of cross section of the tube.
  struct LookupRecord {
    float coord[2];
    float norm[2];
  } * lookup = new LookupRecord[numTubeSections];
  
  for (tubeSection = 0; tubeSection < numTubeSections; ++tubeSection) {
    float tubeAngle = 2.0f*PI*(tubeSectionStart+tubeSection)/tubeSections;
    lookup[tubeSection].norm[0] = sin(tubeAngle);
    lookup[tubeSection].norm[1] = cos(tubeAngle);
    lookup[tubeSection].coord[0] = ringRadius + tubeRadius*lookup[tubeSection].norm[0];
    lookup[tubeSection].coord[1] = z + tubeRadius*lookup[tubeSection].norm[1];
  }
  
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  float ringAngle = 0.0f;
  float direction[2] = { 0.0f, 1.0f };
  for (ringSection = 0; ringSection <= ringSections; ++ringSection) {
    // copy old direction vector
    float direction2[2] = { direction[0], direction[1] };
    // calculate new angle and direction vector
    ringAngle = -2.0f*PI*ringSection/ringSections;
    direction[0] = sin(ringAngle);
    direction[1] = cos(ringAngle);
    glBegin(GL_TRIANGLE_STRIP);
    {
      // draw section of torus between new and old values
      for (tubeSection = 0; tubeSection < numTubeSections; ++tubeSection) {
        float outwards = lookup[tubeSection].coord[0];
        float upwards = lookup[tubeSection].coord[1];
        float normSin = lookup[tubeSection].norm[0];
        float normCos = lookup[tubeSection].norm[1];
        glNormal3f(direction[0]  * normSin,  direction[1]  * normSin,  normCos);
        glVertex3f(direction[0]  * outwards, direction[1]  * outwards, upwards);
        glNormal3f(direction2[0] * normSin,  direction2[1] * normSin,  normCos);
        glVertex3f(direction2[0] * outwards, direction2[1] * outwards, upwards);
      }
    }
    glEnd();
  }
  
  // Clean up member from lookup table
  delete [] lookup;
}


void Drawing::Disk(float radius, unsigned int sections, float z, bool flip)
{
  float normZ = flip ? -1.0f : 1.0f;
  glNormal3f(0.0f, 0.0f, normZ);
  glBegin(GL_TRIANGLE_FAN);
  for (unsigned int i = 0; i < sections; ++i) {
    float ang = normZ*2.0f*PI * i / sections;
    glVertex3f(radius*cos(ang), radius*sin(ang), z);
  }
  glEnd();
}

void Drawing::ThickDisk(float radius, float thickness, unsigned int sections, float z)
{
  Torus(radius, thickness, sections, sections >> 1, z, 0, sections >> 2);
  Disk(radius, sections, z + thickness, false);
  Disk(radius, sections, z - thickness, true);
}

void Drawing::Cylinder(float topRadius, float bottomRadius,
                       float topZ, float bottomZ,
                       unsigned int ringSections,
                       bool capTop, bool capBottom)
{
  glBegin(GL_TRIANGLE_STRIP);
  {
    for (unsigned int i = 0; i <= ringSections; ++i) {
      float angle = 2.0f*PI*i/ringSections;
      float sinAngle = sin(angle);
      float cosAngle = cos(angle);
      glNormal3f(cosAngle, sinAngle, 0.0f);
      glVertex3f(topRadius * cosAngle, topRadius * sinAngle, topZ);
      glVertex3f(bottomRadius * cosAngle, bottomRadius * sinAngle, bottomZ);
    }
  }
  glEnd();
  if (capTop) {
    Disk(topRadius, ringSections, topZ, false);
  }
  if (capBottom) {
    Disk(bottomRadius, ringSections, bottomZ, true);
  }
}

void Drawing::Arrow(float lineRadius, float headRadius, float headLength, float lineEnd, float lineStart, int segments)
{
  int i;
  // The line
  glBegin(GL_TRIANGLE_FAN);
  {
    glNormal3f(0.0f, 0.0f, -1.0f);
    for (i = 0; i < segments; ++i) {
      float ang = 2.0f*PI*i/segments;
      float sinAng = sin(ang);
      float cosAng = cos(ang);
      glVertex3f(lineRadius*sinAng, lineRadius*cosAng, lineStart);
    }
  }
  glEnd();
  glBegin(GL_TRIANGLE_STRIP);
  {
    for (i = 0; i <= segments; ++i) {
      float ang = 2.0f*PI*i/segments;
      float sinAng = sin(ang);
      float cosAng = cos(ang);
      glNormal3f(sinAng, cosAng, 0.0f);
      glVertex3f(lineRadius*sinAng, lineRadius*cosAng, lineStart);
      glVertex3f(lineRadius*sinAng, lineRadius*cosAng, lineEnd-headLength);
    }
  }
  glEnd();
  
  // The head
  glBegin(GL_TRIANGLE_STRIP);
  {
    glNormal3f(0.0f, 0.0f, -1.0f);
    for (i = 0; i <= segments; ++i) {
      float ang = 2.0f*PI*i/segments;
      float sinAng = sin(ang);
      float cosAng = cos(ang);
      glVertex3f(lineRadius*sinAng, lineRadius*cosAng, lineEnd-headLength);
      glVertex3f(headRadius*sinAng, headRadius*cosAng, lineEnd-headLength);
    }
  }
  glEnd();
  // Draw as a triangle strip so the lighting looks ok if used
  glBegin(GL_TRIANGLE_STRIP);
  {
    float headAng = atan2(headRadius, headLength);
    float sinHeadAng = sin(headAng);
    float cosHeadAng = cos(headAng);
    for (i = 0; i <= segments; ++i) {
      float ang = 2.0f*PI*i/segments;
      float sinAng = sin(ang);
      float cosAng = cos(ang);
      glNormal3f(sinAng * sinHeadAng, cosAng * sinHeadAng, cosHeadAng);
      glVertex3f(headRadius*sinAng, headRadius*cosAng, lineEnd-headLength);
      glVertex3f(0.0f, 0.0f, lineEnd);
    }
  }
  glEnd();
}
