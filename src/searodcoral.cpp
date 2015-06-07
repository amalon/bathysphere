/*
 * searodcoral.cpp
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
 * Sea Rod Coral generater.
 *
 */

#include "searodcoral.h"

#include <cstdlib>

/// Constructor.
SeaRodCoral::SeaRodCoral(float size)
{
  // Make some initial values.
  float radius = 0.05f + 0.03f * rand() / RAND_MAX;
  maths::Vector<3, float> position(0.0f, 0.0f, 0.0f);
  
  // Compile the display list.
  BeginCompile();
#define STRANDS 3
  for (int i = 0; i < STRANDS; ++i) {
    float angle = 2.0f*PI*i/STRANDS;
    Physics::Orientation orientation;
    orientation.ApplyRotation(angle*180.0f/PI, maths::Vector<3, float>(0.0f, 0.0f, 1.0f));
    orientation.ApplyRotation(40.0f, maths::Vector<3, float>(1.0f, 0.0f, 0.0f));
    maths::Vector<3, float> angularChange(sin(angle), cos(angle), 0.0f);
    //angularChange *= 0.1f;
    CoralConstruction(radius,
                      position,
                      orientation,
                      angularChange,
                      5, 20);
  }
  EndCompile();
}
#include "primitives.h"
#include <iostream>

/// Construct and draw an arm of the coral
void SeaRodCoral::CoralConstruction(float radius,
                                    maths::Vector<3, float> position,
                                    Physics::Orientation orientation,
                                    maths::Vector<3, float> angularChange,
                                    int maxDepth, int maxLength)
{
#define SECTIONS 8
#define STRIPS (SECTIONS >> 2)
  // Generate the coral
  /*
  go from position in the +ve Z of orientation
  gradually curve according to angularChange
  apply small changes to angularChange randomly
  occasionally split to the side
  */
#define ACTION_GROW 0
#define ACTION_SPLIT 1
#define ACTION_STOP 2
  
  int normalGrows = 0;
  bool reroll = true;
  int action = 0; // silence warning, reroll is true so always set before read
  while (true) {
    if (reroll) {
      int dice = rand() % 100;
      //std::cout << dice << std::endl;
      if (dice < 10) {
        action = ACTION_STOP;
      } else if (dice < 30) {
        action = ACTION_SPLIT;
      } else {
        action = ACTION_GROW;
      }
    } else {
      reroll = true;
    }
    
    if (action == ACTION_GROW) {
      if (!maxLength) {
        normalGrows = 500;
        action = ACTION_STOP;
        reroll = false;
        continue;
      }
      --maxLength;
      ++normalGrows;
      
      // Just carry on without splitting.
      maths::Vector<3, float> nextPosition = position;
      nextPosition += orientation.GetMatrix() * maths::Vector<3, float>(0.0f, 0.0f, 3.0f*radius);
      angularChange += maths::Vector<3, float>( -1.0f + (float)rand() / RAND_MAX,
                                                -1.0f + (float)rand() / RAND_MAX,
                                                -1.0f + (float)rand() / RAND_MAX) * 0.1f;
      
      Physics::Orientation nextOrientation = orientation;
      nextOrientation.ApplyRotation(radius, angularChange);
      
      // Start to point upwards
      maths::Quaternion<float> identity;
      identity.setIdentity();
      maths::Quaternion<float> toUp = slerp(orientation.GetQuaternion(), identity, 0.1f);
      nextOrientation.SetQuaternion(toUp);
      
      // Tweak the radius slightly
      float nextRadius = radius * (0.8f + 0.4f*rand() / RAND_MAX);
      
      glBegin(GL_TRIANGLE_STRIP);
      for (int i = 0; i <= SECTIONS; ++i) {
        float angle = 2.0f*PI*i/SECTIONS;
        float sinAngle = sin(angle);
        float cosAngle = cos(angle);
        
        maths::Vector<3, float> norm1 = orientation.GetMatrix() * maths::Vector<3, float>(sinAngle, cosAngle, 0.0);
        glTexCoord2f((float)i/SECTIONS, 0.0f);
        glNormal(norm1);
        glVertex3(position + norm1*radius);
        
        maths::Vector<3, float> norm2 = nextOrientation.GetMatrix() * maths::Vector<3, float>(sinAngle, cosAngle, 0.0);
        glTexCoord2f((float)i/SECTIONS, 1.0f);
        glNormal(norm2);
        glVertex3(nextPosition + norm2*nextRadius);
      }
      glEnd();
      
      orientation = nextOrientation;
      position = nextPosition;
      radius = nextRadius;
    }
    
    if (action == ACTION_SPLIT) {
      if (normalGrows < 3) {
        action = ACTION_GROW;
        reroll = false;
        continue;
      }
      if (!maxDepth) {
        continue;
      }
      normalGrows = 0;
      // Split
      Physics::Orientation newOrientation = orientation;
      float randomAngle = 2.0f * PI * rand() / RAND_MAX;
      maths::Vector<3, float> randomAxis(sin(randomAngle), cos(randomAngle), 0.0f);
      //orientation.ApplyRotation(-10.0f*rand() / RAND_MAX, randomAxis);
      angularChange += randomAxis * -0.2f;
      newOrientation.ApplyRotation(90.0f + 30.0f*rand() / RAND_MAX, randomAxis);
      CoralConstruction(radius*0.9f, position, newOrientation, angularChange, maxDepth-1, maxLength);
    }
    
    if (action == ACTION_STOP) {
      if (normalGrows < 5) {
        action = ACTION_GROW;
        reroll = false;
        continue;
      }
      // End here
      // Draw a cap and finish
      glPushMatrix();
      glTranslate(position);
      orientation.ApplyGlTranformation();
      int alternator = 0;
      float sincosValues[2][2];
      sincosValues[alternator][0] = 0.0f;
      sincosValues[alternator][1] = 1.0f;
      alternator = 1 - alternator;
      for (int i = 1; i <= STRIPS; ++i) {
        float angle = 0.5f * PI * i / STRIPS;
        sincosValues[alternator][0] = sin(angle);
        sincosValues[alternator][1] = cos(angle);
        glBegin(GL_TRIANGLE_STRIP);
        {
          for (int j = 0; j <= SECTIONS; ++j) {
            float angle2 = 2*PI*j/SECTIONS;
            float sinAngle2 = sin(angle2);
            float cosAngle2 = cos(angle2);
            
            glTexCoord2f((float)j/SECTIONS, (float)i/STRIPS);
            glNormal3f(sinAngle2*sincosValues[1-alternator][1],
                       cosAngle2*sincosValues[1-alternator][1],
                       sincosValues[1-alternator][0]);
            glVertex3f(radius * sinAngle2*sincosValues[1-alternator][1],
                       radius * cosAngle2*sincosValues[1-alternator][1],
                       radius * sincosValues[1-alternator][0]);
            
            glTexCoord2f((float)j/SECTIONS, (float)i/STRIPS);
            glNormal3f(sinAngle2*sincosValues[alternator][1],
                       cosAngle2*sincosValues[alternator][1],
                       sincosValues[alternator][0]);
            glVertex3f(radius * sinAngle2*sincosValues[alternator][1],
                       radius * cosAngle2*sincosValues[alternator][1],
                       radius * sincosValues[alternator][0]);
          }
        }
        glEnd();
        alternator = 1 - alternator;
      }
      glPopMatrix();
      break;
    }
  }
}
