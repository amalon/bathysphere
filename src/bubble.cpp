/*
 * bubble.cpp
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
 * Bubble object.
 *
 */

#include "bubble.h"
#include "observer.h"
#include "environment.h"
#include "glmaths.h"

#include <cstdlib>
#include <iostream>

Bubble::Bubble(float radius)
{
  _alpha = true;
  _collisions = true;
  // Model as a sphere of air
  _radius = radius;
  _volume = (4.0f/3.0f*PI)*radius*radius*radius;
  _density = 1.2f; // aprox 1.2kg/m3
}

/// Find whether the bubble has hit the water yet.
float Bubble::VDetectCollision(Collision::Interaction & result,
                               const maths::Vector<3, float> & prepos, const maths::Vector<3, float> & postpos)
{
  maths::Vector<2, float> pos2(postpos);
  float alt = Object::s_defaultEnvironment->seaSurface->GetAltitude(pos2);
  if (prepos[2] > alt) {
    Delete();
    result.normal.set(0.0f, 0.0f, 1.0f);
    result.location = prepos;
    return 0.0f;
  }
  if (postpos[2] > alt) {
    Delete();
    result.normal.set(0.0f, 0.0f, 1.0f);
    float f = (alt - prepos[2]) / (postpos[2] - prepos[2]);
    result.location = prepos + (postpos-prepos) * f;
    return f;
  }
  return NO_COLLISION;
}

#define BUBBLE_STABLE_LIMIT 0.05f
#define BUBBLE_UNSTABLE_LIMIT 0.2f
float Bubble::GetStability()
{
  // Determine the bubble stability from the radius
  if (GetRadius() <= BUBBLE_STABLE_LIMIT) {
    return 1.0f;
  } else {
    if (GetRadius() < BUBBLE_UNSTABLE_LIMIT) {
      return (BUBBLE_UNSTABLE_LIMIT - GetRadius()) / (BUBBLE_UNSTABLE_LIMIT - BUBBLE_STABLE_LIMIT);
    } else {
      return 0.0f;
    }
  }
}

float Bubble::GetSquash()
{
  // Determine the bubble squashness from the velocity through the liquid.
  float squash = GetVelocity()[2];
  if (squash < 0.0f) {
    squash = -squash;
  }
  squash = 1.0f - 0.5f*squash;
  if (squash < 0.5f) {
    squash = 0.5f;
  } else if (squash > 1.0f) {
    squash = 1.0f;
  }
  return squash;
}

float Bubble::GetFringeAngle(float stability)
{
  return PI * (0.5f + 0.5f * stability);
}

void Bubble::GetFringeCircle(float stability, float squash, float & z, float & radius)
{
  float ang = GetFringeAngle(stability);
  z = GetRadius() * squash * cos(ang);
  radius = GetRadius() * sin(ang);
}

void Bubble::VAdvance(float dt)
{
  float stability = GetStability();
  
  // Now, depending on the stability of the bubble, generate new bubbles on the fringe and within.
  if (stability < 1.0f) {
    float squash = GetSquash();
    
    float z, radius;
    GetFringeCircle(stability, squash, z, radius);
    
    if (stability < 1.0f) {
      for (int i = 0; i < 2; ++i) {
        float randomRadius = GetRadius() * 0.2f * rand() / RAND_MAX;
        float randomAng = 2.0f * PI * rand() / RAND_MAX;
        float randomPosRadius = radius - randomRadius;
        Bubble * subBubble = new Bubble(randomRadius);
        subBubble->SetPosition(GetPosition() +
            maths::Vector<3, float>(randomPosRadius*sin(randomAng),
                                    randomPosRadius*cos(randomAng),
                                        z));
        subBubble->SetVelocity(GetVelocity());
        Object::s_defaultEnvironment->AddObject(subBubble);
        _volume -= subBubble->_volume;
        _radius = cbrt(_volume/(4.0f/3.0f*PI));
      }
    }
  }
  
  // Apply a random force to mix it up a bit
  ApplyForce(maths::Vector<3, float>(-1.0f + 2.0f * rand()/RAND_MAX,
                                     -1.0f + 2.0f * rand()/RAND_MAX,
                                     -1.0f + 2.0f * rand()/RAND_MAX) * (GetMass() * 1.0f));
}

// Render the object
void Bubble::VRender(const Observer & observer)
{
  // Metal
  {
    static float diffuse[4]  = {0.5f, 0.5f, 0.5f, 0.6f};
    static float specular[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 128.0f);
  }
  
  // How stable the bubble is
  float stability = GetStability();
  // How squashed the bubble is
  float squash = GetSquash();
  
  // We're gonna be doing scaling so ensure normals are scaled.
  glEnable(GL_RESCALE_NORMAL);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  // Scale by the squash factor so that the bubble is squashed
  glPushMatrix();
  {
    glScalef(GetRadius(), GetRadius(), GetRadius()*squash);
    //glScalef(5,5,5);
    
    int rings = 3 + (int)(10.0f*GetRadius());
    int sections = rings << 1;
    int ring, section;
    
    float ringsMax = GetFringeAngle(stability);
    
    // The main bubble
    float prevAng = 0.0f;
    float prevSinAng = sin(prevAng);
    float prevCosAng = cos(prevAng);
    for (ring = 1; ring <= rings; ++ring) {
      float ang = ringsMax * ring / rings;
      float sinAng = sin(ang);
      float cosAng = cos(ang);
      glBegin(GL_TRIANGLE_STRIP);
      {
        for (section = 0; section <= sections; ++section) {
          float sectionAng = 2.0f*PI*section/sections;
          float sinSectionAng = sin(sectionAng);
          float cosSectionAng = cos(sectionAng);
          glNormal3f(sinAng*sinSectionAng,     sinAng*cosSectionAng,     cosAng);
          glVertex3f(sinAng*sinSectionAng,     sinAng*cosSectionAng,     cosAng);
          glNormal3f(prevSinAng*sinSectionAng, prevSinAng*cosSectionAng, prevCosAng);
          glVertex3f(prevSinAng*sinSectionAng, prevSinAng*cosSectionAng, prevCosAng);
        }
      }
      glEnd();
      prevAng = ang;
      prevSinAng = sinAng;
      prevCosAng = cosAng;
    }
    
    // The inside of the bubble if its unstable
    float offset = prevCosAng*2.0f;
    for (ring = 1; ring <= rings>>1; ++ring) {
      float ang = ringsMax + (PI-ringsMax) * ring / (rings>>1);
      float sinAng = sin(ang);
      float cosAng = cos(ang);
      glBegin(GL_TRIANGLE_STRIP);
      {
        for (section = 0; section <= sections; ++section) {
          float sectionAng = 2.0f*PI*section/sections;
          float sinSectionAng = sin(sectionAng);
          float cosSectionAng = cos(sectionAng);
          glNormal3f(-sinAng*sinSectionAng,    -sinAng*cosSectionAng,     cosAng);
          glVertex3f( sinAng*sinSectionAng,     sinAng*cosSectionAng,     offset - cosAng);
          glNormal3f(-prevSinAng*sinSectionAng,-prevSinAng*cosSectionAng, prevCosAng);
          glVertex3f( prevSinAng*sinSectionAng, prevSinAng*cosSectionAng, offset - prevCosAng);
        }
      }
      glEnd();
      prevAng = ang;
      prevSinAng = sinAng;
      prevCosAng = cosAng;
    }
  }
  glPopMatrix();
  
  glDisable(GL_BLEND);
  glDisable(GL_RESCALE_NORMAL);
}
